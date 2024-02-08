package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Aiming;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class TunerConstants {

    private static DoublePreferenceConstant frontLeftEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/FrontLeft/Offset", .33);
    private static DoublePreferenceConstant frontRightEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/FrontRight/Offset", .34);
    private static DoublePreferenceConstant backLeftEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/BackLeft/Offset", .067);
    private static DoublePreferenceConstant backRightEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/BackRight/Offset", -1.045);

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(3).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 12;
    private static final int kFrontLeftSteerMotorId = 13;
    private static final int kFrontLeftEncoderId = 13;
    private static final double kFrontLeftEncoderOffset = frontLeftEncoderOffset.getValue();

    private static final double kFrontLeftXPosInches = Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0;
    private static final double kFrontLeftYPosInches = Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0;

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 2;
    private static final double kFrontRightEncoderOffset = frontRightEncoderOffset.getValue();

    private static final double kFrontRightXPosInches = 11.5;
    private static final double kFrontRightYPosInches = -11.25;

    // Back Left
    private static final int kBackLeftDriveMotorId = 15;
    private static final int kBackLeftSteerMotorId = 14;
    private static final int kBackLeftEncoderId = 14;
    private static final double kBackLeftEncoderOffset = backLeftEncoderOffset.getValue();

    private static final double kBackLeftXPosInches = -11.5;
    private static final double kBackLeftYPosInches = 11.25;

    // Back Right
    private static final int kBackRightDriveMotorId = 0;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 1;
    private static final double kBackRightEncoderOffset = backRightEncoderOffset.getValue();

    private static final double kBackRightXPosInches = -11.5;
    private static final double kBackRightYPosInches = -11.25;

    public static final double[] kModuleOffsets = { kFrontLeftEncoderOffset, kFrontRightEncoderOffset,
            kBackLeftEncoderOffset, kBackRightEncoderOffset };

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_ENCODER, kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_ENCODER, kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_ENCODER, kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static CommandSwerveDrivetrain createDrivetrain(Aiming aiming) {
        return new CommandSwerveDrivetrain(DrivetrainConstants, aiming, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
