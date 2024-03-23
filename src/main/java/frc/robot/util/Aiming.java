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
import frc.robot.ros.bridge.Frames;
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.std_msgs.RosColorRGBA;
import frc.team88.ros.messages.visualization_msgs.Marker;
import frc.team88.ros.messages.visualization_msgs.MarkerArray;
import frc.robot.Constants;

public class Aiming {
    private Pose2d robotPose;
    private Alliance alliance;
    private TFListenerCompact tf_compact;
    private TagSubscriber tagSubscriber;
    private final int[] speakerTagsRed = { 3, 4 };
    private final double speakerHeight = Units.inchesToMeters(60.265913);
    private BridgePublisher<MarkerArray> aimPub;

    private DoublePreferenceConstant p_aimingOffset = new DoublePreferenceConstant("Aiming Offset",
            0.15);

    // TODO get these bounds
    // private final double[] shootingAngleBounds = { 44.0, 26.0 };

    // private final double[] pivotAngleBounds = { 42.0, 80.0 };

    public Trigger isInWing = new Trigger(
            () -> RobotState.isTeleop() && getROSPose().getX() < Units.inchesToMeters(231.20));

    public Aiming() {
    }

    public void setTFListener(TFListenerCompact tfListener) {
        tf_compact = tfListener;
    }

    public void setTagListener(TagSubscriber tagsub) {
        tagSubscriber = tagsub;
    }

    public void setAimPub(BridgePublisher<MarkerArray> array) {
        aimPub = array;
    }

    public void sendTarget() {
        Marker marker = new Marker();
        marker.setHeader(aimPub.getHeader(Frames.BASE_FRAME));
        marker.setAction(Marker.ADD);
        marker.setFrameLocked(false);
        marker.setPose(ROSConversions.wpiToRosPose(aimPose()));
        marker.setType(Marker.ARROW);
        marker.setScale(new Vector3(0.05, 0.05, 0.5));
        marker.setColor(new RosColorRGBA(1.0f, 0.0f, 0.0f, 1.0f));
        aimPub.send(new MarkerArray(new Marker[] { marker }));
    }

    public double mapValue(double x, double min, double max, double newMin, double newMax) {
        return (max - min) / (newMax - newMin) * (x - newMin) + min;
    }

    public Pose2d getROSPose() {
        Optional<Transform3dStamped> tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
        if (tfStamped.isEmpty()) {
            return new Pose2d();
        }
        robotPose = new Pose2d(tfStamped.get().transform.getTranslation().toTranslation2d(),
                tfStamped.get().transform.getRotation().toRotation2d());
        return (getAlliance() == DriverStation.Alliance.Red) ? DriveUtils.redBlueTransform(robotPose) : robotPose;
    }

    public double getSpeakerAngleForDrivetrian() {
        Pose2d robotPose = getROSPose();
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE)
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE);
        double drivetrainAngle = Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
        // if(robotPose.getTranslation().getNorm() > ) {
        // drivetrainAngle -= robotPose.getTranslation().getNorm() * 0.13;
        // }
        return drivetrainAngle;
    }

    public double getOdomSpeakerAngleForDrivetrian(Pose2d odomPose) {
        Pose2d robotPose = odomPose;
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE)
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE);
        double drivetrainAngle = Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
        // if(robotPose.getTranslation().getNorm() > ) {
        // drivetrainAngle -= robotPose.getTranslation().getNorm() * 0.13;
        // }
        return drivetrainAngle;
    }

    public double speakerAngleForShooter() {
        Pose2d robotPose = getROSPose();
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

    public double getAmpAngleForDrivetrain() {
        Pose2d robotPose = getROSPose();
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_AMP_POSE)
                : robotPose.relativeTo(Constants.BLUE_AMP_POSE);
        double drivetrainAmpAngle = Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
        return drivetrainAmpAngle;
    }

    public double getDumpingGroundAngle() {
        Pose2d robotPose = getROSPose();
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.DUMPING_GROUND_RED)
                : robotPose.relativeTo(Constants.DUMPING_GROUND_BLUE);
        double drivetrainAmpAngle = Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
        return drivetrainAmpAngle;
    }

    public boolean getDetections() {
        try {
            var header = tagSubscriber.receive().get().getHeader();
            if (header.getFrameId() == "optical_camera_0") {
                var detections = tagSubscriber.receive().get().getDetections();
                for (var detection : detections) {
                    ArrayList<Integer> ids = detection.getId();
                    if (ids.contains(speakerTagsRed[0]) && ids.contains(speakerTagsRed[1])) {
                        return true;
                    }
                }
            }

        } catch (Exception exception) {
            return false;
        }
        return false;
    }

    public Pose3d aimPose() {
        Pose2d robotPose = getROSPose();
        robotPose = (getAlliance() == DriverStation.Alliance.Red) ? Constants.RED_SPEAKER_POSE.relativeTo(robotPose)
                : Constants.BLUE_SPEAKER_POSE.relativeTo(robotPose);
        return new Pose3d(robotPose.getX(), robotPose.getY(), speakerHeight, new Rotation3d());
    }

    public double speakerDistance() {
        Pose2d robotPose = getROSPose();
        return (getAlliance() == DriverStation.Alliance.Red)
                ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE).getTranslation().getNorm()
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE).getTranslation().getNorm();
    }

    // originPoint should be relative to the origin of whatever alliance we are on
    public double getAngletoAnyPoint(Pose2d originPoint) {
        Pose2d robotPose = getROSPose();
        robotPose = robotPose.relativeTo(originPoint);
        return Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
    }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        return alliance;
    }

    public Trigger isInWing() {
        return isInWing;
    }
}
