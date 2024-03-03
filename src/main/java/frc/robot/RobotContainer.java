// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.messages.visualization_msgs.MarkerArray;

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
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Aiming;
import frc.robot.subsystems.Intake;

public class RobotContainer {

    // private DoublePreferenceConstant p_autoCloseAim = new
    // DoublePreferenceConstant("Auto/AutoCloseAim", 63.0);

    private final Aiming m_aiming = new Aiming();
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandGenericHID buttonBox = new CommandGenericHID(1); // The buttons???
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_aiming); // My drivetrain
    private String m_autoCommandName = "Wait";
    private final Shooter m_shooter = new Shooter();
    private final Elevator m_elevator = new Elevator();
    private final Intake m_intake = new Intake(m_elevator::areElevatorAndPivotDown);
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

        // PathPlanner Named Commands
        NamedCommands.registerCommand("Prep Shooter", m_shooter.runShooterFactory());
        NamedCommands.registerCommand("Shoot", new WaitUntilCommand(m_shooter::isShooterAtFullSpeed)
                .andThen(m_intake.shootIndexerFactory().withTimeout(0.5)));
        NamedCommands.registerCommand("Localize", drivetrain.localizeFactory());
        NamedCommands.registerCommand("Intake", m_intake.intakeFactory());
        NamedCommands.registerCommand("Pivot Stow", m_elevator.stowFactory());
        // NamedCommands.registerCommand("Pivot Aim",
        // m_elevator.goToAnlgeFactory(p_autoCloseAim.getValue())
        // .until(() -> m_elevator.pivotOnTarget(p_autoCloseAim.getValue(), 2)));
        // NamedCommands.registerCommand("Pivot Aim",
        // m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
        // .until(() -> m_elevator.pivotOnTarget(m_aiming.speakerAngleForShooter(),
        // 2.0)));
        NamedCommands.registerCommand("Aim",
                new ParallelCommandGroup(m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
                        .until(() -> m_elevator.pivotOnTarget(m_aiming.speakerAngleForShooter(), 2.0)),
                        drivetrain.aimAtSpeakerFactory().until(drivetrain::onTarget)));
        NamedCommands.registerCommand("Stop Shooter", m_shooter.stopShooterFactory().withTimeout(0.2));

        configureSmartDashboardButtons();

        // set default commands
        drivetrain.setDefaultCommand(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)));
        m_shooter.setDefaultCommand(m_shooter.stopShooterFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        m_intake.setDefaultCommand(m_intake.stopMovingFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        m_elevator.setDefaultCommand(m_elevator.stowFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        drivetrain.resetPose(new Pose2d());
        m_climber.setDefaultCommand(m_climber.stowArmFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
    }

    private void configureRosNetworkTablesBridge() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startClient3("coprocessor");
        instance.setServer("10.0.88.44", 5800);

        ROSNetworkTablesBridge bridge = new ROSNetworkTablesBridge(instance.getTable(""), 20);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        TagSubscriber tagsub = new TagSubscriber(bridge);
        BridgePublisher<MarkerArray> aimPub = new BridgePublisher<>(bridge, "target_aiming");
        coprocessorBridge = new CoprocessorBridge(drivetrain, bridge, tfListenerCompact);
        m_aiming.setTFListener(tfListenerCompact);
        m_aiming.setTagListener(tagsub);
        m_aiming.setAimPub(aimPub);
    }

    private void configureDriverController() {
        drivetrain.tipping().whileTrue(m_climber.holdPositionFactory()).whileTrue(m_elevator.holdPositionFactory());
        m_shooter.shooterAtSpeed().onTrue(setRumble());
        joystick.b().onTrue(drivetrain.setHeadingFactory(270));
        joystick.x().onTrue(drivetrain.setHeadingFactory(90));
        joystick.y().onTrue(drivetrain.setHeadingFactory(0));
        joystick.a().onTrue(drivetrain.setHeadingFactory(180));
        isRightStickZero().debounce(0.25, DebounceType.kRising)
                .onTrue(drivetrain.setHeadingFactory(() -> drivetrain.getState().Pose.getRotation().getDegrees()))
                .whileFalse(drivetrain.applyRequest(drivetrain.fieldCentricRequest(joystick)));
        joystick.rightTrigger()
                .onTrue(m_intake.shootIndexerFactory()
                        .unless(() -> drivetrain.tipping().getAsBoolean() || !m_intake.hasNoteInIndexer()));
        joystick.rightBumper()
                .whileTrue(
                        m_shooter.runShooterFactory().alongWith(new WaitUntilCommand(m_shooter::isShooterAtFullSpeed))
                                .andThen(setRumble()).unless(() -> drivetrain.tipping().getAsBoolean()))
                .whileTrue(drivetrain.aimAtSpeakerFactory().unless(() -> drivetrain.tipping().getAsBoolean()))
                .whileTrue(m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
                        .unless(() -> drivetrain.tipping().getAsBoolean() || !m_intake.hasNoteInIndexer()));
    }

    private void configureButtonBox() {
        m_intake.hasNote().onTrue((m_shooter.runIdleSpeedFactory()).unless(() -> !m_intake.m_automaticMode))
                .onTrue(setRumble().unless(() -> !m_intake.m_automaticMode));
        m_intake.hasNote().and(() -> !m_intake.m_automaticMode)
                .onFalse(m_intake.intakeFactory().alongWith(m_shooter.stopShooterFactory())).debounce(0.25);
        buttonBox.button(10).whileTrue(m_intake.intakeFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(20)
                .whileTrue(m_intake.shootIndexerFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(18).whileTrue(m_intake.rejectFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(17).whileFalse(m_shooter.stopShooterFactory());
        buttonBox.button(5).whileTrue(m_elevator.setPodiumFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(6)
                .whileTrue(m_elevator.setAmpFactory().alongWith(m_shooter.slowSpeedFactory())
                        .until(() -> m_elevator.pivotOnTargetForAmp() && m_elevator.elevatorOnTarget())
                        .andThen(m_shooter.runShooterFactory()).unless(() -> drivetrain.tipping().getAsBoolean()))
                .onFalse(m_elevator.elevatorDownFactory()
                        .until(() -> m_elevator.elevatorOnTarget() && m_elevator.pivotOnTarget(42.0, 2.0))
                        .unless(() -> drivetrain.tipping().getAsBoolean()))
                .onFalse(m_shooter.stopShooterFactory().unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(11).onTrue(m_climber.stowArmFactory().alongWith(m_elevator.stowFactory())
                .unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(2).onTrue(m_climber.prepArmsFactory().alongWith(m_elevator.elevatorPrepFactory())
                .unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(15)
                .onTrue(new SequentialCommandGroup(
                        m_elevator.climbFactory().alongWith(m_climber.prepArmsFactory())
                                .until(m_elevator::elevatorOnTarget),
                        m_climber.climbFactory().alongWith(m_elevator.climbFactory()))
                        .unless(() -> drivetrain.tipping().getAsBoolean()));
        buttonBox.button(19).onTrue(m_climber.softLandingFactory().alongWith(m_elevator.climbFactory())
                .unless(() -> drivetrain.tipping().getAsBoolean()))
                .onFalse(new InstantCommand(() -> m_intake.enableAutoMode()));
        buttonBox.button(8)
                .onTrue(new InstantCommand(() -> m_intake.disableAutoMode()).andThen(new WaitCommand(0.1))
                        .andThen(new ParallelCommandGroup(
                                m_elevator.trapFactory(), m_climber.climbFactory(),
                                m_shooter.slowSpeedFactory().until(m_elevator::pivotOnTargetForAmp)
                                        .andThen(m_shooter.runAmpTrapSpeedFactory().withTimeout(1.5))
                                        .andThen(m_intake.shootIndexerFactory())))
                        .unless(() -> drivetrain.tipping().getAsBoolean()));
        // buttonBox.button(16).whileTrue(m_elevator.goToAimingPosition(() ->
        // m_aiming.speakerAngleForShooter()));
    }

    private void configureSmartDashboardButtons() {
        // Drive
        SmartDashboard.putData("SetLowPowerMode", drivetrain.lowPowerModeFactory());
        SmartDashboard.putData("SetHighPowerMode", drivetrain.highPowerModeFactory());
        SmartDashboard.putData("Localize", drivetrain.localizeFactory().ignoringDisable(true));

        // Shooter
        // SmartDashboard.putData("Run Shooter", m_shooter.runShooterCommand());
        // SmartDashboard.putData("Stop Shooter", m_shooter.stopShooterCommand());

        // Elevator
        SmartDashboard.putData("Calibrate Pivot", m_elevator.calibratePivotFactory());
        SmartDashboard.putData("Calibrate Elevator", m_elevator.calibrateElevatorFactory());
        SmartDashboard.putData("Go to Stow", m_elevator.stowFactory());
        SmartDashboard.putData("Go To Flat", m_elevator.setFlatFactory());

        // Climber
        SmartDashboard.putData("ClimberCalibrate", m_climber.calibrateFactory());
        SmartDashboard.putData("ClimberCoastMode", m_climber.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ClimberBrakeMode", m_climber.enableBrakeModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorCoastMode", m_elevator.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorBrakeMode", m_elevator.enableBrakeModeFactory().ignoringDisable(true));

        // Auto Test
        SmartDashboard.putData("Red Line Auto", drivetrain.getAutoPath("TwoPieceAuto"));
        SmartDashboard.putData("Four Piece", drivetrain.getAutoPath("FourPiece"));
        SmartDashboard.putData("Rumble", setRumble().ignoringDisable(true));
    }

    private void configureBindings() {
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Trigger isRightStickZero() {
        return new Trigger(() -> RobotState.isTeleop() && Math.abs(joystick.getRightX()) < 0.01
                && Math.abs(joystick.getRightY()) < 0.01);
    }

    public void teleopInit() {
        drivetrain.setTargetHeading(drivetrain.getState().Pose.getRotation().getDegrees());
        m_intake.intakeFactory().schedule();
    }

    public void disabledPeriodic() {
        if (buttonBox.button(12).getAsBoolean() && !m_autoCommandName.equals("FourPiece")) {
            m_autoCommand = drivetrain.getAutoPath("FourPiece");
            m_autoCommandName = "FourPiece";
        }
        if ((buttonBox.button(8)).getAsBoolean() && !m_autoCommandName.equals("SixPiece")) {
            m_autoCommand = drivetrain.getAutoPath("SixPiece");
            m_autoCommandName = "SixPiece";
        }

        if (buttonBox.button(13).getAsBoolean()) {
            m_autoCommand = new WaitCommand(15);
            m_autoCommandName = "Waiting";
        }
        SmartDashboard.putString("Auto", m_autoCommandName);
    }

    private Command setRumble() {
        return new SequentialCommandGroup(new InstantCommand(
                () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                new WaitCommand(1),
                new InstantCommand(() -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    }

    public void autonomousInit() {
        m_elevator.calibratePivot();
        // drivetrain.localize();
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
