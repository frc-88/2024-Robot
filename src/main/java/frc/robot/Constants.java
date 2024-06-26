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

    public static final String RIO_CANBUS = "rio";
    public static final String CANIVORE_CANBUS = "1";

    ///////////////////////////////////////////////////////
    // DRIVETRAIN
    ///////////////////////////////////////////////////////

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from module center to module center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_INCHES = 22.75;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from module center to module center.
     */
    public static final double DRIVETRAIN_WHEELBASE_INCHES = 19.75;

    public static final double DRIVETRAIN_CENTER_OFFSET = 2.3125;
    public static final String DRIVETRAIN_CANBUS = CANIVORE_CANBUS;
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 6;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2;

    public static final double TIPPING_THRESHOLD = 20.0;

    // Intake
    public static final int INTAKE_MOTOR_ID = 14;
    public static final int INTAKE_GUIDE_MOTOR_ID = 15;
    public static final int INTAKE_INDEX_MOTOR_ID = 16;
    public static final int INTAKE_CURRENT_LIMIT = 20;

    // Elevator
    public static final int ELEVATOR_ANGLER_MOTOR = 10;
    public static final int ELEVATOR_MOTOR = 13;
    public static final int ELEVATOR_PIGEON_ID = 0;
    public static final double ELEVATOR_BOTTOM = 27.7;
    public static final double PIVOT_BOTTOM = 42.0;

    // Shooter
    public static final int SHOOTER_LEFT_MOTOR = 11;
    public static final int SHOOTER_RIGHT_MOTOR = 12;
    public static final int SHOOTER_CURRENT_LIMIT = 80;

    // Climber
    public static final int CLIMBER_LEFT_MOTOR = 9;
    public static final int CLIMBER_RIGHT_MOTOR = 0;

    // Lights
    public static final int CANDLE_ID = 6;

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
            .redBlueTransform(new Pose2d(new Translation2d((16.54 - 0.12), 5.547868),
                    Rotation2d.fromDegrees(180)));
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(new Translation2d(0.12, 5.547868),
            Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_AMP_AIM_POSE = new Pose2d(new Translation2d(1.83, 7.65),
            Rotation2d.fromDegrees(0));
    public static final Pose2d RED_AMP_AIM_POSE = DriveUtils.redBlueTransform(new Pose2d(new Translation2d(14.7, 7.65),
            Rotation2d.fromDegrees(180)));
    public static final Pose2d DUMPING_GROUND_BLUE = new Pose2d(new Translation2d(6.7, 7.55),
            Rotation2d.fromDegrees(0));
    public static final Pose2d DUMPING_GROUND_RED = DriveUtils
            .redBlueTransform(new Pose2d(new Translation2d(10.00, 7.55), Rotation2d.fromDegrees(180)));
    public static final Pose2d BLUE_AMP_PATH_POSE = new Pose2d(new Translation2d(1.83, 7.65),
            Rotation2d.fromDegrees(-90));
    public static final Pose2d RED_AMP_PATH_POSE = DriveUtils
            .redBlueTransform(new Pose2d(new Translation2d(15.25, 8.05),
                    Rotation2d.fromDegrees(-90)));

}
