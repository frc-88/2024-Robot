package frc.robot.util;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.ros.bridge.Frames;
//import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.std_msgs.RosColorRGBA;
import frc.team88.ros.messages.visualization_msgs.Marker;
import frc.team88.ros.messages.visualization_msgs.MarkerArray;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class Aiming {
    private Pose2d robotPose;
    private Alliance alliance;
    // private final int[] speakerTagsRed = { 3, 4 };
    private final double speakerHeight = Units.inchesToMeters((60.265913 - 2.5));

    private DoublePreferenceConstant p_aimingOffset = new DoublePreferenceConstant("Aiming Offset",
            0.11);

    // private LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
    // DriverStation.Alliance.Red)
    // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
    // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // TODO get these bounds
    // private final double[] shootingAngleBounds = { 44.0, 26.0 };

    // private final double[] pivotAngleBounds = { 42.0, 80.0 };

    // public Trigger isInWing = new Trigger(
    // () -> RobotState.isTeleop() && (m_aiming.speakerDistance(robotPose) <
    // Units.feetToMeters(15)));

    public Aiming() {
    }

    public double mapValue(double x, double min, double max, double newMin,
            double newMax) {
        return (max - min) / (newMax - newMin) * (x - newMin) + min;
    }

    // public LimelightHelpers.PoseEstimate limelightPeriodic(double degrees, double
    // rate) {
    // return m_Limelight.limelightPeriodic(degrees, rate);
    // }

    public double getSpeakerAngleForDrivetrian(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE)
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE);
        double drivetrainAngle = Math.atan2(robotPose.getY(), robotPose.getX()) *
                (180 / Math.PI);
        // if(robotPose.getTranslation().getNorm() > ) {
        // drivetrainAngle -= robotPose.getTranslation().getNorm() * 0.13;
        // }
        return drivetrainAngle;
    }

    public double speakerAngleForShooter(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        double distance = (getAlliance() == DriverStation.Alliance.Red)
                ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE).getTranslation().getNorm()
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE).getTranslation().getNorm();

        double shootingAngle = Math.atan2(distance, speakerHeight) * (180 / Math.PI);
        distance = Units.metersToFeet(distance);

        shootingAngle -= distance * p_aimingOffset.getValue(); // aim higher based on
        // double shootingAngle = 19.2 + (6.03 * distance) - (0.171 * distance *
        // distance);

        if (shootingAngle < 42) {
            shootingAngle = 42;
        }

        return shootingAngle;
    }

    public double odomSpeakerAngle(Pose2d odomPose) {
        Pose2d robotPose = odomPose;
        double distance = (getAlliance() == DriverStation.Alliance.Red)
                ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE).getTranslation().getNorm()
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE).getTranslation().getNorm();

        double shootingAngle = Math.atan2(distance, speakerHeight) * (180 / Math.PI);
        distance = Units.metersToFeet(distance);

        shootingAngle -= distance * p_aimingOffset.getValue(); // aim higher based on distance
        // double shootingAngle = 19.2 + (6.03 * distance) - (0.171 * distance *
        // distance);

        if (shootingAngle < 42) {
            shootingAngle = 42;
        }

        return shootingAngle;
    }

    public double getAmpAngleForDrivetrain(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_AMP_AIM_POSE)
                : robotPose.relativeTo(Constants.BLUE_AMP_AIM_POSE);
        double drivetrainAmpAngle = Math.atan2(robotPose.getY(), robotPose.getX()) *
                (180 / Math.PI);
        return drivetrainAmpAngle;
    }

    public double getDumpingGroundAngle(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.DUMPING_GROUND_RED)
                : robotPose.relativeTo(Constants.DUMPING_GROUND_BLUE);
        double drivetrainAmpAngle = Math.atan2(robotPose.getY(), robotPose.getX()) *
                (180 / Math.PI);
        return drivetrainAmpAngle;
    }

    public boolean getDetections() {
        try {
            // var header = tagSubscriber.receive().get().getHeader();
            // if (header.getFrameId() == "optical_camera_0") {
            // var detections = tagSubscriber.receive().get().getDetections();
            // for (var detection : detections) {
            // ArrayList<Integer> ids = detection.getId();
            // if (ids.contains(speakerTagsRed[0]) && ids.contains(speakerTagsRed[1])) {
            // return true;
            // }
            // }
            // }

        } catch (Exception exception) {
            return false;
        }
        return false;
    }

    public Pose3d aimPose(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? Constants.RED_SPEAKER_POSE.relativeTo(robotPose)
                : Constants.BLUE_SPEAKER_POSE.relativeTo(robotPose);
        return new Pose3d(robotPose.getX(), robotPose.getY(), speakerHeight, new Rotation3d());
    }

    public double speakerDistance(Pose2d pose) {
        // Pose2d robotPose = getROSPose();
        // LimelightHelpers.PoseEstimate robotPosemt = (getAlliance() ==
        // DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight")
        // : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        // Pose2d robotPose = robotPosemt.pose;
        Pose2d robotPose = pose;
        return (getAlliance() == DriverStation.Alliance.Red)
                ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE).getTranslation().getNorm()
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE).getTranslation().getNorm();
    }

    // originPoint should be relative to the origin of whatever alliance we are

    public double getAngletoAnyPoint(Pose2d originPoint) {
        // Pose2d robotPose = getROSPose();
        robotPose = robotPose.relativeTo(originPoint);
        return Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
    }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        return alliance;
    }

    // public Trigger isInWing() {
    // return isInWing;
    // }
}
