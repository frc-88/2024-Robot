package frc.robot.ros.bridge;

import java.util.Optional;

import org.opencv.core.Mat;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ros.bridge.AprilTagDetectionArray;
import frc.robot.util.DriveUtils;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.geometry_msgs.Pose2D;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovarianceStamped;

public class AprilTagPoseSubscriber implements Subscriber<PoseWithCovarianceStamped> {
    private final BridgeSubscriber<PoseWithCovarianceStamped> tagSub;
    private Pose2d lastPose;
    private int m_nsecs;
    private Double[] m_visionCovariance;
    private SwerveDrivetrain m_drivetrain;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable tagPoseTable = inst.getTable("AprilTagPose");
    private final DoubleArrayPublisher tagFieldPub = tagPoseTable.getDoubleArrayTopic("robotPose")
            .publish();
    private final StringPublisher tagFieldTypePub = tagPoseTable.getStringTopic(".type").publish();

    public AprilTagPoseSubscriber(ROSNetworkTablesBridge bridge, SwerveDrivetrain drivetrain) {
        tagSub = new BridgeSubscriber<>(bridge, "/northstar/landmark", PoseWithCovarianceStamped.class);
        m_drivetrain = drivetrain;
    }

    public Optional<PoseWithCovarianceStamped> receive() {
        Optional<PoseWithCovarianceStamped> msg;
        if ((msg = tagSub.receive()).isPresent()) {
            lastPose = toPose2d(msg.get());
            lastPose = new Pose2d(lastPose.getX(), lastPose.getY(),
                    lastPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
            lastPose = DriveUtils.redAlliance() ? DriveUtils.redBlueTransform(lastPose)
                    : lastPose;
            sendPose(lastPose);
            m_nsecs = msg.get().getHeader().getStamp().getNsecs();

            m_visionCovariance = msg.get().getPose().getCovariance();
            Matrix<N3, N1> m_visionMatrix = new Matrix<N3, N1>(N3.instance, N1.instance);
            m_visionMatrix.set(0, 0, m_visionCovariance[0] * 10);
            m_visionMatrix.set(1, 0, m_visionCovariance[7] * 10);
            m_visionMatrix.set(2, 0, m_visionCovariance[35] * 1000);

            SmartDashboard.setDefaultNumberArray("Vision Co-variance", m_visionCovariance);

            m_drivetrain.addVisionMeasurement(lastPose, Timer.getFPGATimestamp(),
                    m_visionMatrix);
        }
        return null;
    }

    public Pose2d getLastTag() {
        receive();
        return lastPose;
    }

    private void sendPose(Pose2d pose) {
        if (DriveUtils.redAlliance()) {
            pose = DriveUtils.redBlueTransform(pose);
        }
        tagFieldTypePub.set("Field2d");
        tagFieldPub.set(new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
        });
    }

    private Pose2d toPose2d(PoseWithCovarianceStamped pose) {
        return new Pose2d(pose.getPose().getPose().getPosition().getX(), pose.getPose().getPose().getPosition().getY(),
                Rotation2d.fromRotations(pose.getPose().getPose().getOrientation().getZ()));
    }

}