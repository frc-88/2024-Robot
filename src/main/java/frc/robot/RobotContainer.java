// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.ros.bridge.CoprocessorBridge;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Aiming;

public class RobotContainer {
    private final Aiming m_aiming = new Aiming();
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_aiming); // My drivetrain
    private String m_autoCommandName = "Wait";
    private Command m_autoCommand = new WaitCommand(15);

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps, drivetrain);
    private TFListenerCompact tfListenerCompact;
    @SuppressWarnings("unused")
    private CoprocessorBridge coprocessorBridge;

    public RobotContainer() {
        DataLogManager.start();
        configureRosNetworkTablesBridge();
        configureDriverController();
        configureBindings();

        // PathPlanner Named Commands
        NamedCommands.registerCommand("Prep Shooter", new WaitCommand(1));
        NamedCommands.registerCommand("Shoot", new WaitCommand(1));
        NamedCommands.registerCommand("Intake", new WaitCommand(1));
        NamedCommands.registerCommand("Localize", drivetrain.localizeFactory());
        NamedCommands.registerCommand("ResetHeading",
                drivetrain.setHeadingFactory(() -> drivetrain.getState().Pose.getRotation().getDegrees()));

        configureSmartDashboardButtons();

        // set default commands
        drivetrain.setDefaultCommand(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)));

        drivetrain.resetPose(new Pose2d());
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
        joystick.rightBumper().whileTrue(drivetrain.aimAtSpeakerFactory());
        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(drivetrain.brakeRequest()));
        // reset the field-centric heading on left bumper press
        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> {
            drivetrain.resetPose(new Pose2d());
        }));
        joystick.leftBumper().whileTrue(drivetrain.aimAtSpeakerFactory());
    }

    private void configureSmartDashboardButtons() {
        // Drive
        SmartDashboard.putData("SetLowPowerMode", drivetrain.lowPowerModeFactory());
        SmartDashboard.putData("SetHighPowerMode", drivetrain.highPowerModeFactory());
        SmartDashboard.putData("Localize", drivetrain.localizeFactory());

        // Auto Test
        SmartDashboard.putData("Zero Odometry", drivetrain.zeroOdomFactory());
        SmartDashboard.putData("Red Line Auto", drivetrain.getAutoPath("TwoPieceAuto"));
        SmartDashboard.putData("Race Auto", drivetrain.getAutoPath("RaceAuto"));
        SmartDashboard.putData("FourPiece", drivetrain.getAutoPath("FourPiece"));
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
    }

    public void disabledPeriodic() {
        if (joystick.button(1).getAsBoolean() && !m_autoCommandName.equals("TwoPieceAuto")) {
            m_autoCommand = drivetrain.getAutoPath("TwoPieceAuto");
            m_autoCommandName = "TwoPieceAuto";
        }
        if (joystick.button(2).getAsBoolean() && !m_autoCommandName.equals("ThreePieceAuto")) {
            m_autoCommand = drivetrain.getAutoPath("ThreePieceAuto");
            m_autoCommandName = "ThreePieceAuto";
        }

        if (joystick.button(3).getAsBoolean()) {
            m_autoCommand = new WaitCommand(15);
            m_autoCommandName = "Waiting";
        }
        SmartDashboard.putString("Auto", m_autoCommandName);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
