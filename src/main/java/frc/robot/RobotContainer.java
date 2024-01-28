// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.util.DriveUtils;

public class RobotContainer {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final SlewRateLimiter filterY = new SlewRateLimiter(500);
    private final SlewRateLimiter filterX = new SlewRateLimiter(500);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();
    private final PhoenixPIDController headingController = new PhoenixPIDController(10, 0, 0) {
        @Override
        public double calculate(double measurement, double setpoint, double currentTimestamp) {
            double output = super.calculate(measurement, setpoint, currentTimestamp);
            output = MathUtil.clamp(output, -MaxAngularRate / 2, MaxAngularRate / 2);
            return output;
        }
    };

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        filterX.calculate(DriveUtils.signedPow(joystick.getLeftX() * MaxSpeed, 2)))
                                .withVelocityY(
                                        -filterY.calculate(DriveUtils.signedPow(joystick.getLeftY() * MaxSpeed, 2)))
                                .withRotationalRate(DriveUtils.signedPow(-joystick.getRightX() * MaxAngularRate, 2))));

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        snapToAngle.HeadingController = headingController;

        joystick.b().whileTrue(drivetrain.applyRequest(getSnapToAngleRequest(270)));
        joystick.x().whileTrue(drivetrain.applyRequest(getSnapToAngleRequest(90)));
        joystick.y().whileTrue(drivetrain.applyRequest(getSnapToAngleRequest(0)));
        joystick.a().whileTrue(drivetrain.applyRequest(getSnapToAngleRequest(180)));
        isNotMoving().whileTrue(drivetrain
                .applyRequest(() -> pointWheelsAt.withModuleDirection(Rotation2d.fromDegrees(
                        (drivetrain.getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble() * 360)))));

        joystick.rightTrigger()
                .whileTrue(drivetrain.applyRequest(
                        () -> robotCentric.withVelocityX(filterX.calculate(joystick.getLeftX() * MaxSpeed))
                                .withVelocityY(-filterY.calculate(joystick.getLeftY() * MaxSpeed))
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
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

    private Supplier<SwerveRequest> getSnapToAngleRequest(double degrees) {
        return () -> snapToAngle.withVelocityX(filterX.calculate(joystick.getLeftX() * MaxSpeed))
                .withVelocityY(-filterY.calculate(joystick.getLeftY() * MaxSpeed))
                .withTargetDirection(Rotation2d.fromDegrees(degrees));
    }

    private Trigger isNotMoving() {
        return new Trigger(() -> joystick.getLeftX() == 0 && joystick.getLeftY() == 0 && joystick.getRightX() == 0
                && joystick.getRightY() == 0);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
