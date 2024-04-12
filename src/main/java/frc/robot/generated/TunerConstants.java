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

    // Default offsets below are for dartfrog on 3/7
    public static DoublePreferenceConstant p_frontLeftEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/FrontLeft/Offset", -0.056152);
    public static DoublePreferenceConstant p_frontRightEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/FrontRight/Offset", -0.415527);
    public static DoublePreferenceConstant p_backLeftEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/BackLeft/Offset", 0.070557);
    public static DoublePreferenceConstant p_backRightEncoderOffset = new DoublePreferenceConstant(
            "swervedrive/BackRight/Offset", 0.112793);
    private static DoublePreferenceConstant p_driveCurrentLimit = new DoublePreferenceConstant("Drive Current Limit",
            80);
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
    private static final double kSlipCurrentA = p_driveCurrentLimit.getValue();

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 5.04;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2.03;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(Constants.DRIVETRAIN_PIGEON_ID)
            .withCANbusName(Constants.DRIVETRAIN_CANBUS);

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

    public static final double[] kModuleOffsets = {
            p_frontLeftEncoderOffset.getValue(), p_frontRightEncoderOffset.getValue(),
            p_backLeftEncoderOffset.getValue(), p_backRightEncoderOffset.getValue() };

    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_LEFT_MODULE_STEER_ENCODER, p_frontLeftEncoderOffset.getValue(),
            Units.inchesToMeters((Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0) - Constants.DRIVETRAIN_CENTER_OFFSET),
            Units.inchesToMeters(Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0),
            kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, p_frontRightEncoderOffset.getValue(),
            Units.inchesToMeters((Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0) - Constants.DRIVETRAIN_CENTER_OFFSET),
            Units.inchesToMeters(-Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0),
            kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.BACK_LEFT_MODULE_STEER_ENCODER, p_backLeftEncoderOffset.getValue(),
            Units.inchesToMeters((-Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0) - Constants.DRIVETRAIN_CENTER_OFFSET),
            Units.inchesToMeters(Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0),
            kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.BACK_RIGHT_MODULE_STEER_ENCODER, p_backRightEncoderOffset.getValue(),
            Units.inchesToMeters((-Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0) - Constants.DRIVETRAIN_CENTER_OFFSET),
            Units.inchesToMeters(-Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0),
            kInvertRightSide);

    public static CommandSwerveDrivetrain createDrivetrain(Aiming aiming) {
        return new CommandSwerveDrivetrain(DrivetrainConstants, aiming, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
