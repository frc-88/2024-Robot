package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Aiming;
import frc.robot.util.DriveUtils;
import frc.robot.util.HoldAnglesRequest;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 * 
 * why did the robot
 * cross the road? electronics!
 * that's why we did it
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
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

    private boolean holdingDirections = false;
    private DoublePreferenceConstant p_maxVeloctiy = new DoublePreferenceConstant("drivetrain/PathFindingMaxVelocity",
            5.21);
    private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant(
            "drivetrain/PathFindingMaxAcceleration", 3.0);
    private DoublePreferenceConstant p_maxAngularVelocity = new DoublePreferenceConstant(
            "drivetrain/PathFindingMaxAngularVelocity", 540.0);
    private DoublePreferenceConstant p_maxAngularAcceleration = new DoublePreferenceConstant(
            "drivetrain/PathFindingMaxAngularAcceleration", 720);

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private DoublePreferenceConstant p_tippingRollThreshold = new DoublePreferenceConstant("Tipping Roll Threashold",
            8.0);
    private DoublePreferenceConstant p_tippingPitchThreshold = new DoublePreferenceConstant("Tipping Pitch Threashold",
            15.0);

    /* Robot pose for field positioning */
    private final NetworkTable rosPoseTable = inst.getTable("ROSPose");
    private final DoubleArrayPublisher rosFieldPub = rosPoseTable.getDoubleArrayTopic("robotPose")
            .publish();
    private final StringPublisher rosFieldTypePub = rosPoseTable.getStringTopic(".type").publish();
    private final NetworkTable odomTable = inst.getTable("Pose");
    private final DoubleArrayPublisher poseFieldPub = odomTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher poseFieldTypePub = odomTable.getStringTopic(".type").publish();

    private Trigger m_tipping = new Trigger(() -> tippingPitch() || tippingRoll());

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final HoldAnglesRequest holdAngles = new HoldAnglesRequest();
    private final SwerveRequest.FieldCentricFacingAngle snapToAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();
    private final PhoenixPIDController headingController = new PhoenixPIDController(12.0, 0, 1.0) {
        @Override
        public double calculate(double measurement, double setpoint, double currentTimestamp) {
            double output = super.calculate(measurement, setpoint, currentTimestamp);
            output = MathUtil.clamp(output, -MaxAngularRate, MaxAngularRate);
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

    public Command defaultDriveCommand(CommandXboxController joystick) {
        return run(() -> {
            if (Math.abs(joystick.getLeftX()) < .05 && Math.abs(joystick.getLeftY()) < .05
                    && Math.abs(joystick.getRightX()) < 0.05
                    && Math.abs(getCurrentRobotAngle() - targetHeading) < 1) {
                if (!holdingDirections) {
                    Rotation2d directions[] = new Rotation2d[4];
                    for (int i = 0; i < 4; i++) {
                        directions[i] = getModule(i).getCurrentState().angle;
                    }
                    holdAngles.setModuleDirections(directions);
                    holdingDirections = true;
                }
                this.setControl(holdAngles);
            } else {
                this.setControl(this.SnapToAngleRequest(joystick).get());
                holdingDirections = false;
            }
        });
    }

    public Command getAutoPath(String pathName) {
        try {
            Command autoPath = new PathPlannerAuto(pathName);
            return autoPath;
        } catch (Exception e) {
            Command autoPath = new WaitCommand(1.0);
            System.err.println("Exception loading auto path");
            e.printStackTrace();
            return autoPath;
        }

    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetPose(Pose2d pose) {
        seedFieldRelative(pose);
        setTargetHeading(pose.getRotation().getDegrees());
    }

    // returns the current pose in blue coordinates
    public Pose2d getPoseBlue() {
        if (DriveUtils.redAlliance())
            return DriveUtils.redBlueTransform(getState().Pose);
        else
            return getState().Pose;
    }

    // pass in a pose in blue coordinates
    public void resetPoseBlue(Pose2d pose) {
        if (DriveUtils.redAlliance())
            pose = DriveUtils.redBlueTransform(pose);
        seedFieldRelative(pose);
        setTargetHeading(pose.getRotation().getDegrees());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        this.setControl(autoRequest.withSpeeds(speeds));
    }

    private void configureAutoBuilder() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(this::getPoseBlue, this::resetPoseBlue,
                this::getChassisSpeeds, this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(new PIDConstants(10.0, 0.0, 0.0), // Translational constant
                        new PIDConstants(9.0, 0.0, 0.3), // Rotational constant
                        TunerConstants.kSpeedAt12VoltsMps, // in m/s
                        driveBaseRadius, // in meters
                        new ReplanningConfig()),
                DriveUtils::redAlliance,
                this);
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
                    .withRotationalRate(DriveUtils.signedPow(-angularRate, 2) * MaxAngularRate);
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

    public Supplier<SwerveRequest> autoSnapToAngleRequest() {
        return () -> {
            return snapToAngle.withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withTargetDirection(Rotation2d.fromDegrees(targetHeading));
        };
    }

    public Supplier<SwerveRequest> idleRequest() {
        return () -> idle;
    }

    public Supplier<SwerveRequest> pointWheelsAtRequest() {
        return () -> pointWheelsAt.withModuleDirection(
                Rotation2d.fromDegrees(getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble() * 360));
    }

    public void setOffsetsToZero() {
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.MagnetOffset = 0;
        for (int i = 0; i < 4; i++) {
            getModule(i).getCANcoder().getConfigurator().apply(configuration);
        }
    }

    public void setOffsets() {
        TunerConstants.p_frontLeftEncoderOffset
                .setValue(getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble());
        TunerConstants.p_frontRightEncoderOffset
                .setValue(getModule(1).getCANcoder().getAbsolutePosition().getValueAsDouble());
        TunerConstants.p_backLeftEncoderOffset
                .setValue(getModule(2).getCANcoder().getAbsolutePosition().getValueAsDouble());
        TunerConstants.p_backRightEncoderOffset
                .setValue(getModule(3).getCANcoder().getAbsolutePosition().getValueAsDouble());
    }

    public void localize() {
        resetPose(m_aiming.getROSPose());
    }

    public double getCurrentRobotAngle() {
        return getState().Pose.getRotation().getDegrees();
    }

    public boolean onTarget() {
        return getCurrentRobotAngle() - targetHeading < 2.0;
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

    public Command zeroOdomFactory() {
        return new InstantCommand(
                () -> resetPose(new Pose2d(Units.inchesToMeters(Constants.DRIVETRAIN_WHEELBASE_INCHES / 2.0),
                        Units.inchesToMeters(Constants.DRIVETRAIN_TRACKWIDTH_INCHES / 2.0),
                        new Rotation2d())));
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

    public Command aimAtAmpDumpingGroundFactory(BooleanSupplier amp) {
        return new RunCommand(() -> setTargetHeading(
                amp.getAsBoolean() ? m_aiming.getAmpAngleForDrivetrain() : m_aiming.getDumpingGroundAngle()));
    }

    private void sendROSPose() {
        /* Telemeterize the pose */
        Pose2d pose = m_aiming.getROSPose();
        if (DriveUtils.redAlliance()) {
            pose = DriveUtils.redBlueTransform(pose);
        }
        rosFieldTypePub.set("Field2d");
        rosFieldPub.set(new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });
    }

    public Command pathFindingCommand(Supplier<Pose2d> targetPose) {
        Pose2d pose = DriveUtils.redAlliance()
                ? DriveUtils.redBlueTransform(targetPose.get())
                : targetPose.get();
        PathConstraints constraints = new PathConstraints(p_maxVeloctiy.getValue(), p_maxAcceleration.getValue(),
                Units.degreesToRadians(p_maxAngularVelocity.getValue()),
                Units.degreesToRadians(p_maxAngularAcceleration.getValue()));

        return AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0);
    }

    private void sendOdomPose() {
        Pose2d pose = getState().Pose;
        if (DriveUtils.redAlliance()) {
            pose = DriveUtils.redBlueTransform(pose);
        }
        poseFieldTypePub.set("Field2d");
        poseFieldPub.set(new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });
    }

    public Trigger drivetrainOnTarget() {
        return new Trigger(() -> onTarget());
    }

    public Command calibrateFactory() {
        return new SequentialCommandGroup(new InstantCommand(this::setOffsetsToZero), new WaitCommand(5),
                new InstantCommand(this::setOffsets));
    }

    public boolean tippingRoll() {
        return Math.abs(getPigeon2().getRoll().getValue()) >= p_tippingRollThreshold.getValue();
    }

    public boolean tippingPitch() {
        return Math.abs(getPigeon2().getPitch().getValue()) >= p_tippingPitchThreshold.getValue();
    }

    public Trigger tipping() {
        return m_tipping;
    }

    @Override
    public void periodic() {
        sendROSPose();
        sendOdomPose();
        SmartDashboard.putNumber("Target Heading", targetHeading);
        SmartDashboard.putNumber("Speaker Angle", m_aiming.getSpeakerAngleForDrivetrian());
        SmartDashboard.putNumber("Speaker Distance", Units.metersToFeet(m_aiming.speakerDistance()));
        SmartDashboard.putBoolean("Tag sub", m_aiming.getDetections());
        SmartDashboard.putNumber("Shooter Aiming", m_aiming.speakerAngleForShooter());
        SmartDashboard.putNumber("Pigeon Yaw", getPigeon2().getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll", getPigeon2().getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Pitch", getPigeon2().getPitch().getValueAsDouble());
        SmartDashboard.putString("PowerMode", lowPowerMode ? "LowPowerMode" : "HighPowerMode");
        SmartDashboard.putNumber("AmpShuttle", m_aiming.getAmpAngleForDrivetrain());
        m_aiming.sendTarget();
    }
}
