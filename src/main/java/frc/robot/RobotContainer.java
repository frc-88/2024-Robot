// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private void configureBindings() {

        drivetrain.setDefaultCommand(drivetrain.applyRequest(drivetrain.fieldCentricRequest(joystick))); // Drivetrain
                                                                                                         // will execute
                                                                                                         // this command
                                                                                                         // periodically

        joystick.b().whileTrue(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick, 270)));
        joystick.x().whileTrue(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick, 90)));
        joystick.y().whileTrue(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick, 0)));
        joystick.a().whileTrue(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick, 180)));
        isNotMoving().whileTrue(drivetrain.applyRequest(drivetrain.pointWheelsAtRequest()));
        isRightStickZero()
                .whileTrue(drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick, getCurrentRobotAngle())));
        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(drivetrain.robotCentricRequest(joystick)));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(drivetrain.brakeRequest()));
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldRelative();
            drivetrain.setRobotOffset();
        }));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Trigger isNotMoving() {
        return new Trigger(() -> joystick.getLeftX() == 0 && joystick.getLeftY() == 0 && joystick.getRightX() == 0
                && joystick.getRightY() == 0);
    }

    private Trigger isRightStickZero() {
        return new Trigger(() -> joystick.getRightX() == 0 && joystick.getRightY() == 0 && joystick.getLeftX() != 0
                && joystick.getLeftY() != 0);
    }

    private double getCurrentRobotAngle() {
        return drivetrain.getState().Pose.getRotation().getDegrees();
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
