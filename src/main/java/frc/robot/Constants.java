// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.DriveUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    ///////////////////////////////////////////////////////
    // DRIVETRAIN
    ///////////////////////////////////////////////////////

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from module center to module center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_INCHES = 22.5; // TODO get from CAD
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from module center to module center.
     */
    public static final double DRIVETRAIN_WHEELBASE_INCHES = 23.0; // TODO get from CAD

    public static final String DRIVETRAIN_CANBUS = "";
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 15;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;

    // Intake

    // Elevator

    // Shooter

    // Climber

    // Lights

    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int BUTTON_BOX_ID = 1;
    public static final int TEST_CONTROLLER_ID = 2;

    // Coprocessor
    public static final String COPROCESSOR_ADDRESS = "10.0.88.44";
    public static final int COPROCESSOR_PORT = 5800;
    public static final double COPROCESSOR_UPDATE_DELAY = 1.0 / 30;
    public static final double COPROCESSOR_UPDATE_DELAY_OFFSET = 0.01;
    public static final double COPROCESSOR_SLOW_UPDATE_DELAY = 1.0 / 5;
    public static final double COPROCESSOR_SLOW_UPDATE_DELAY_OFFSET = 0.02;

    // Aiming
    public static final Pose2d RED_SPEAKER_POSE = DriveUtils
            .redBlueTransform(new Pose2d(new Translation2d(16.579342, 5.547868),
                    Rotation2d.fromDegrees(180)));
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(new Translation2d(-0.0381, 5.547868),
            Rotation2d.fromDegrees(0));
}
