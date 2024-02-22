// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.ros.bridge.CoprocessorBridge;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Aiming;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private final Aiming m_aiming = new Aiming();
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandGenericHID buttonBox = new CommandGenericHID(1); // The buttons???
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_aiming); // My drivetrain
    private String m_autoCommandName = "Wait";

    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    private final Elevator m_elevator = new Elevator();
    private Climber m_climber = new Climber();

    private Command m_autoCommand = new SequentialCommandGroup(
            new WaitCommand(4),
            m_intake.shootIndexerFactory().withTimeout(2),
            new RunCommand(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds(1, 0, 0)), drivetrain).withTimeout(2),
            drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)))
            .alongWith(m_shooter.runShooterFactory().withTimeout(6).andThen(m_shooter.stopShooterFactory()));

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps, drivetrain);
    private TFListenerCompact tfListenerCompact;
    @SuppressWarnings("unused")
    private CoprocessorBridge coprocessorBridge;

    public RobotContainer() {
        DataLogManager.start();
        configureRosNetworkTablesBridge();
        configureDriverController();
        configureButtonBox();
        configureBindings();
        configureSmartDashboardButtons();

        // PathPlanner Named Commands
        NamedCommands.registerCommand("Prep Shooter", m_shooter.runShooterFactory());
        NamedCommands.registerCommand("Shoot", m_intake.shootIndexerFactory());
        NamedCommands.registerCommand("Localize", drivetrain.localizeFactory());
        NamedCommands.registerCommand("Intake", m_intake.intakeFactory());

        // set default commands
        drivetrain.setDefaultCommand(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)));
        m_shooter.setDefaultCommand(m_shooter.runIdleSpeedFactory());
        m_intake.setDefaultCommand(m_intake.stopMovingFactory());
        m_elevator.setDefaultCommand(m_elevator.stowFactory());
        drivetrain.resetPose(new Pose2d());
        m_climber.setDefaultCommand(m_climber.stowArmFactory());
    }

    private void configureRosNetworkTablesBridge() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        ROSNetworkTablesBridge bridge = new ROSNetworkTablesBridge(instance.getTable(""), 20);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        coprocessorBridge = new CoprocessorBridge(drivetrain, bridge, tfListenerCompact);
        m_aiming.setTFListener(tfListenerCompact);
    }

    private void configureDriverController() {
        joystick.b().onTrue(drivetrain.setHeadingFactory(270));
        joystick.x().onTrue(drivetrain.setHeadingFactory(90));
        joystick.y().onTrue(drivetrain.setHeadingFactory(0));
        joystick.a().onTrue(drivetrain.setHeadingFactory(180));
        isRightStickZero().debounce(0.25, DebounceType.kRising)
                .onTrue(drivetrain.setHeadingFactory(() -> drivetrain.getState().Pose.getRotation().getDegrees()))
                .whileFalse(drivetrain.applyRequest(drivetrain.fieldCentricRequest(joystick)));
        // joystick.rightTrigger().whileTrue(drivetrain.applyRequest(drivetrain.robotCentricRequest(joystick)));
        joystick.rightTrigger().whileTrue(m_intake.shootIndexerFactory());
        joystick.rightBumper().whileTrue(m_shooter.runShooterFactory()).whileTrue(drivetrain.aimAtSpeakerFactory());
        joystick.leftBumper().and(joystick.rightBumper()).whileFalse(
                buttonBox.button(17).getAsBoolean() ? m_shooter.runIdleSpeedFactory()
                        : m_shooter.stopShooterFactory());
        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(drivetrain.brakeRequest()));
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().whileTrue(m_shooter.runShooterFactory());
    }

    private void configureButtonBox() {
        buttonBox.button(10).whileTrue(m_intake.intakeFactory());
        buttonBox.button(20).whileTrue(m_intake.shootIndexerFactory());
        buttonBox.button(18).whileTrue(m_intake.rejectFactory());
        buttonBox.button(17).whileFalse(m_shooter.stopShooterFactory());
        buttonBox.button(5).whileTrue(m_elevator.setPodiumFactory());
        buttonBox.button(6).whileTrue(m_elevator.setAmpFactory());
        buttonBox.button(8).whileTrue(m_climber.setPositionFactory());
        buttonBox.button(11).onTrue(m_climber.stowArmFactory().alongWith(m_elevator.stowFactory()));
        buttonBox.button(2).onTrue(m_climber.prepArmsFactory().alongWith(m_elevator.elevatorPrepFactory()));
        buttonBox.button(15)
                .whileTrue(new SequentialCommandGroup(
                        m_elevator.climbFactory().alongWith(m_climber.prepArmsFactory()).until(m_elevator::onTarget),
                        m_climber.climbFactory().alongWith(m_elevator.climbFactory())))
                .onFalse(m_climber.softLandingFactory().alongWith(m_elevator.climbFactory()));
        buttonBox.button(3).whileTrue(m_elevator.goToAimingPosition());

    }

    private void configureSmartDashboardButtons() {
        // Drive
        SmartDashboard.putData("SetLowPowerMode", drivetrain.lowPowerModeFactory());
        SmartDashboard.putData("SetHighPowerMode", drivetrain.highPowerModeFactory());
        SmartDashboard.putData("Localize", drivetrain.localizeFactory());

        // Shooter
        // SmartDashboard.putData("Run Shooter", m_shooter.runShooterCommand());
        // SmartDashboard.putData("Stop Shooter", m_shooter.stopShooterCommand());

        // Elevator
        SmartDashboard.putData("Calibrate Pivot", m_elevator.calibratePivotFactory());
        SmartDashboard.putData("Calibrate Elevator", m_elevator.calibrateElevatorFactory());
        SmartDashboard.putData("Go to Stow", m_elevator.stowFactory());
        SmartDashboard.putData("Go To Flat", m_elevator.setFlatFactory());

        // Climber
        SmartDashboard.putData("ClimberGoToPostition", m_climber.setPositionFactory());
        SmartDashboard.putData("ClimberCalibrate", m_climber.calibrateFactory());
        SmartDashboard.putData("ClimberCoastMode", m_climber.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ClimberBrakeMode", m_climber.enableBrakeModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorCoastMode", m_elevator.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorBrakeMode", m_elevator.enableBrakeModeFactory().ignoringDisable(true));

        // Auto Test
        SmartDashboard.putData("Red Line Auto", drivetrain.getAutoPath("TwoPieceAuto"));
    }

    private void configureBindings() {
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Trigger isRightStickZero() {
        return new Trigger(() -> Math.abs(joystick.getRightX()) < 0.01 && Math.abs(joystick.getRightY()) < 0.01);
    }

    public void teleopInit() {
        drivetrain.setTargetHeading(drivetrain.getState().Pose.getRotation().getDegrees());

        if (buttonBox.button(17).getAsBoolean()) {
            m_shooter.runIdleSpeedFactory().schedule();
        } else {
            m_shooter.stopShooterFactory().schedule();
        }
    }

    public void disabledPeriodic() {
        if (buttonBox.button(12).getAsBoolean() && !m_autoCommandName.equals("TwoPieceAuto")) {
            m_autoCommand = drivetrain.getAutoPath("TwoPieceAuto");
            m_autoCommandName = "TwoPieceAuto";
        }
        if ((buttonBox.button(14)).getAsBoolean() && !m_autoCommandName.equals("ThreePieceAuto")) {
            m_autoCommand = drivetrain.getAutoPath("ThreePieceAuto");
            m_autoCommandName = "ThreePieceAuto";
        }

        if (buttonBox.button(13).getAsBoolean()) {
            m_autoCommand = new WaitCommand(15);
            m_autoCommandName = "Waiting";
        }
        SmartDashboard.putString("Auto", m_autoCommandName);
    }

    public void autonomousInit() {
        m_elevator.calibratePivot();
        drivetrain.localize();
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
