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
import frc.robot.ros.bridge.AprilTagDetectionArray;
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

    public AprilTagPoseSubscriber(ROSNetworkTablesBridge bridge, SwerveDrivetrain drivetrain) {
        tagSub = new BridgeSubscriber<>(bridge, "/northstar/landmark/forwarded", PoseWithCovarianceStamped.class);
        m_drivetrain = drivetrain;
    }

    public Optional<PoseWithCovarianceStamped> receive() {
        PoseWithCovarianceStamped msg;
        if ((msg = tagSub.receive().get()) != null) {
            lastPose = toPose2d(msg);
            m_nsecs = msg.getHeader().getStamp().getNsecs();
            m_visionCovariance = msg.getPose().getCovariance();
            Matrix<N3, N1> m_visionMatrix = new Matrix<N3, N1>(N3.instance, N1.instance);
            m_visionMatrix.set(0, 0, m_visionCovariance[0]);
            m_visionMatrix.set(1, 0, m_visionCovariance[7]);
            m_visionMatrix.set(2, 0, m_visionCovariance[35]);

            m_drivetrain.setVisionMeasurementStdDevs(m_visionMatrix);
            m_drivetrain.addVisionMeasurement(lastPose, m_nsecs / 1.0e6);
        }
        return Optional.ofNullable(msg);
    }

    public Pose2d getLastTag() {
        receive();
        return lastPose;
    }

    private Pose2d toPose2d(PoseWithCovarianceStamped pose) {
        return new Pose2d(pose.getPose().getPose().getPosition().getX(), pose.getPose().getPose().getPosition().getY(), Rotation2d.fromDegrees(pose.getPose().getPose().getOrientation().getZ()));
    }
    
}