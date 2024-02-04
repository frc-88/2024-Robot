package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ros.bridge.Frames;
import frc.robot.util.DriveUtils;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 * 
 * why did the chicken
 * cross the road? electronics!
 * that's why we did it
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double m_fieldRelativeOffset = 0;
    private final SlewRateLimiter filterY = new SlewRateLimiter(500);
    private final SlewRateLimiter filterX = new SlewRateLimiter(500);
    private TFListenerCompact tf_compact;
    private Pose2d rosPose;
    private double targetHeading = 0;

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        snapToAngle.HeadingController = headingController;
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        snapToAngle.HeadingController = headingController;
    }

    public void setTargetHeading(double target) {
        targetHeading = target;
    }

    public void setTargetHeading(DoubleSupplier target) {
        targetHeading = target.getAsDouble();
    }

    public void setTFListener(TFListenerCompact tfListener) {
        tf_compact = tfListener;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public double getRobotOffset() {
        return m_fieldRelativeOffset;
    }

    public void setRobotOffset() {
        // m_fieldRelativeOffset = getState().Pose.getRotation().getDegrees();
        getPigeon2().setYaw(0);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Supplier<SwerveRequest> fieldCentricRequest(CommandXboxController controller) {
        return () -> drive.withVelocityX(filterX.calculate(DriveUtils.signedPow(-controller.getLeftY() * MaxSpeed, 2)))
                .withVelocityY(filterY.calculate(DriveUtils.signedPow(-controller.getLeftX() * MaxSpeed, 2)))
                .withRotationalRate(DriveUtils.signedPow(controller.getRightX() * MaxAngularRate, 2));
    }

    public Supplier<SwerveRequest> SnapToAngleRequest(CommandXboxController controller) {
        return () -> snapToAngle.withVelocityX(filterX.calculate(-controller.getLeftY() * MaxSpeed))
                .withVelocityY(filterY.calculate(-controller.getLeftX() * MaxSpeed))
                .withTargetDirection(Rotation2d.fromDegrees(targetHeading));
    }

    public Supplier<SwerveRequest> robotCentricRequest(CommandXboxController controller) {
        return () -> robotCentric.withVelocityX(filterX.calculate(-controller.getLeftY() * MaxSpeed))
                .withVelocityY(-filterY.calculate(-controller.getLeftX() * MaxSpeed))
                .withRotationalRate(-controller.getRightX() * MaxAngularRate);
    }

    public Supplier<SwerveRequest> brakeRequest() {
        return () -> brake;
    }

    public Supplier<SwerveRequest> pointWheelsAtRequest() {
        return () -> pointWheelsAt.withModuleDirection(
                Rotation2d.fromDegrees(getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble() * 360));
    }

    public void localize() {
        Transform3dStamped tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
        Translation2d XYTranslation = tfStamped.transform.getTranslation().toTranslation2d();
        Rotation2d rotation = tfStamped.transform.getRotation().toRotation2d();
        rosPose = new Pose2d(XYTranslation, rotation);
        seedFieldRelative(rosPose);
    }

    public double getCurrentRobotAngle() {
        return getState().Pose.getRotation().getDegrees();
    }

    public Command localizeFactory() {
        return new InstantCommand(() -> {
            localize();
        }, this);
    }

    public Command setHeadingFactory(double target) {
        return new InstantCommand(() -> setTargetHeading(target));
    }

    public Command setHeadingFactory(DoubleSupplier target) {
        return new InstantCommand(() -> setTargetHeading(target));
    }

    @Override
    public void periodic() {
        // Transform3dStamped tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME,
        // Frames.BASE_FRAME);
        // Translation2d XYTranslation =
        // tfStamped.transform.getTranslation().toTranslation2d();
        // Rotation2d rotation = tfStamped.transform.getRotation().toRotation2d();

        // SmartDashboard.putNumber("ROS X Translation", XYTranslation.getX());
        // SmartDashboard.putNumber("ROS Y Translation", XYTranslation.getY());
        // SmartDashboard.putNumber("ROS Rotation", rotation.getDegrees());
        // SmartDashboard.putNumber("Pigeon Yaw",
        // getPigeon2().getYaw().getValueAsDouble());
    }
}
