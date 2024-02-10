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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Aiming;
import frc.robot.util.DriveUtils;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 * 
 * why did the chicken
 * cross the road? electronics!
 * that's why we did it
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double MaxSpeed = 6; // 6 meters per second desired top speed
    private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private double m_fieldRelativeOffset = 0;
    private final SlewRateLimiter filterY = new SlewRateLimiter(500);
    private final SlewRateLimiter filterX = new SlewRateLimiter(500);
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private double targetHeading = 0;
    private Aiming m_aiming;
    private boolean lowPowerMode = false;

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, Aiming aiming,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configureAutoBuilder();
        m_aiming = aiming;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        snapToAngle.HeadingController = headingController;
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, Aiming aiming,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configureAutoBuilder();
        m_aiming = aiming;
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

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setHighPowerMode() {
        lowPowerMode = false;
    }

    public void setLowPowerMode() {
        lowPowerMode = true;
    }

    public double getRobotOffset() {
        return m_fieldRelativeOffset;
    }

    public void setRobotOffset() {
        getPigeon2().setYaw(0);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    private void configureAutoBuilder() {

        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(this::getPose, this::seedFieldRelative, this::getChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(new PIDConstants(10.0, 0.0, 0.0), // Translational constant
                        new PIDConstants(10.0, 0.0, 0.0), // Rotational constant
                        TunerConstants.kSpeedAt12VoltsMps, // in m/s
                        driveBaseRadius, // in meters
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);

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
        return () -> {
            double leftY = lowPowerMode ? (-controller.getLeftY() / 2) : -controller.getLeftY();
            double leftX = lowPowerMode ? (-controller.getLeftX() / 2) : -controller.getLeftX();
            double angularRate = lowPowerMode ? (controller.getRightX() / 2) : controller.getRightX();
            return drive.withVelocityX(filterX.calculate(DriveUtils.signedPow(leftY, 2) * MaxSpeed))
                    .withVelocityY(filterY.calculate(DriveUtils.signedPow(leftX, 2) * MaxSpeed))
                    .withRotationalRate(DriveUtils.signedPow(angularRate, 2) * MaxAngularRate);
        };
    }

    public Supplier<SwerveRequest> SnapToAngleRequest(CommandXboxController controller) {
        return () -> {
            double leftY = lowPowerMode ? (-controller.getLeftY() / 2) : -controller.getLeftY();
            double leftX = lowPowerMode ? (-controller.getLeftX() / 2) : -controller.getLeftX();
            return snapToAngle.withVelocityX(filterX.calculate(DriveUtils.signedPow(leftY, 2) * MaxSpeed))
                    .withVelocityY(filterY.calculate(DriveUtils.signedPow(leftX, 2) * MaxSpeed))
                    .withTargetDirection(Rotation2d.fromDegrees(targetHeading));
        };
    }

    public Supplier<SwerveRequest> brakeRequest() {
        return () -> brake;
    }

    public Supplier<SwerveRequest> pointWheelsAtRequest() {
        return () -> pointWheelsAt.withModuleDirection(
                Rotation2d.fromDegrees(getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble() * 360));
    }

    public void localize() {
        seedFieldRelative(m_aiming.getROSPose());
    }

    public double getCurrentRobotAngle() {
        return getState().Pose.getRotation().getDegrees();
    }

    public Command highPowerModeFactory() {
        return new InstantCommand(() -> setHighPowerMode(), this);
    }

    public Command lowPowerModeFactory() {
        return new InstantCommand(() -> setLowPowerMode(), this);
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

    public Command aimAtSpeakerFactory() {
        return new RunCommand(() -> setTargetHeading(m_aiming.getSpeakerAngleForDrivetrian()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Speaker Angle", m_aiming.getSpeakerAngleForDrivetrian());
        SmartDashboard.putNumber("ROS X Translation", m_aiming.getROSPose().getX());
        SmartDashboard.putNumber("ROS Y Translation", m_aiming.getROSPose().getY());
        SmartDashboard.putNumber("ROS Rotation", m_aiming.getROSPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putString("PowerMode", lowPowerMode ? "LowPowerMode" : "HighPowerMode");
    }
}
