// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final Aiming m_aiming = new Aiming();
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_aiming); // My drivetrain

    private Command runAuto = new WaitCommand(1.0);

    private final Telemetry logger = new Telemetry(MaxSpeed, drivetrain);
    private TFListenerCompact tfListenerCompact;
    @SuppressWarnings("unused")
    private CoprocessorBridge coprocessorBridge;

    public RobotContainer() {
        configureRosNetworkTablesBridge();
        configureDriverController();
        configureBindings();
        configureSmartDashboardButtons();

        // set default commands
        drivetrain.setDefaultCommand(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)));
    }

    private void configureRosNetworkTablesBridge() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        ROSNetworkTablesBridge bridge = new ROSNetworkTablesBridge(instance.getTable(""), 20);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        coprocessorBridge = new CoprocessorBridge(drivetrain, bridge, tfListenerCompact);
        m_aiming.setTFListener(tfListenerCompact);
        SmartDashboard.putData("Localize", drivetrain.localizeFactory());
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
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(drivetrain.brakeRequest()));
        // reset the field-centric heading on left bumper press
        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> {
            drivetrain.getPigeon2().setYaw(0);
        }));
        joystick.leftBumper().whileTrue(drivetrain.aimAtSpeakerFactory());
    }

    private void configureSmartDashboardButtons() {
    }

    private void configureBindings() {
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("SetLowPowerMode", drivetrain.lowPowerModeFactory());
        SmartDashboard.putData("SetHighPowerMode", drivetrain.highPowerModeFactory());

        // auto test
        SmartDashboard.putData("Red Line Auto", drivetrain.getAutoPath("TwoPieceAuto"));
    }

    private Trigger isRightStickZero() {
        return new Trigger(() -> Math.abs(joystick.getRightX()) < 0.01 && Math.abs(joystick.getRightY()) < 0.01);
    }

    public void teleopInit() {
        drivetrain.setTargetHeading(drivetrain.getState().Pose.getRotation().getDegrees());
    }

    public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("TwoPieceAuto");
        return AutoBuilder.followPath(path);
    }
}
