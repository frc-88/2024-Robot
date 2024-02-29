package frc.robot.util;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.ros.bridge.Frames;
import frc.robot.ros.bridge.TagSubscriber;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.robot.Constants;

public class Aiming {
    private Pose2d robotPose;
    private Alliance alliance;
    private TFListenerCompact tf_compact;
    private TagSubscriber tagSubscriber;
    private final int[] speakerTagsRed = { 3, 4 };
    private final int[] speakerTagsBlue = { 7, 8 };

    // TODO get these bounds
    // private final double[] shootingAngleBounds = { 44.0, 26.0 };

    // private final double[] pivotAngleBounds = { 42.0, 80.0 };

    public Aiming() {

    }

    public void setTFListener(TFListenerCompact tfListener) {
        tf_compact = tfListener;
    }

    public void setTagListener(TagSubscriber tagsub) {
        tagSubscriber = tagsub;
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
        return Math.atan2(robotPose.getY(), robotPose.getX()) * (180 / Math.PI);
    }

    public double speakerAngleForShooter() {
        Pose2d robotPose = getROSPose();
        double distance = (getAlliance() == DriverStation.Alliance.Red)
                ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE).getTranslation().getNorm()
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE).getTranslation().getNorm();
        // TODO also get this from CAD
        final double speakerHeight = Units.inchesToMeters(60.265913);
        // Speaker height 79.829
        // 23.563087

        double shootingAngle = Math.atan2(distance, speakerHeight) * (180 / Math.PI);
        distance = Units.metersToFeet(distance);

        shootingAngle -= distance * 0.13; // aim higher based on distance
        // double shootingAngle = 19.2 + (6.03 * distance) - (0.171 * distance *
        // distance);

        if (shootingAngle < 42) {
            shootingAngle = 42;
        }

        return shootingAngle;
    }

    public boolean getDetections() {
        try {
            var detections = tagSubscriber.receive().get().getDetections();
            for (var detection : detections) {
                ArrayList<Integer> ids = detection.getId();
                if (ids.contains(speakerTagsRed[0]) && ids.contains(speakerTagsRed[1])) {
                    return true;
                }
            }
        } catch (NoSuchElementException e) {
            return false;
        }

        return false;
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
}
