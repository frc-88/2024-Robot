package frc.robot.ros.bridge;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DriveUtils;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.ROSConversions;
import frc.team88.ros.messages.TimePrimitive;
import frc.team88.ros.messages.geometry_msgs.Point;
import frc.team88.ros.messages.geometry_msgs.Pose;
import frc.team88.ros.messages.geometry_msgs.PoseWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Quaternion;
import frc.team88.ros.messages.geometry_msgs.Twist;
import frc.team88.ros.messages.geometry_msgs.TwistWithCovariance;
import frc.team88.ros.messages.geometry_msgs.Vector3;
import frc.team88.ros.messages.nav_msgs.Odometry;
import frc.team88.ros.messages.std_msgs.RosHeader;

public class OdomPublisher implements Publisher {
    private final CommandSwerveDrivetrain commandDriveTrain;
    private final BridgePublisher<Odometry> odomPub;

    public OdomPublisher(CommandSwerveDrivetrain drive, ROSNetworkTablesBridge bridge) {
        commandDriveTrain = drive;
        odomPub = new BridgePublisher<>(bridge, "odom");
    }

    private final Odometry odomMsg = new Odometry(new RosHeader(0, new TimePrimitive(), Frames.ODOM_FRAME),
            Frames.BASE_FRAME,
            new PoseWithCovariance(new Pose(new Point(0, 0, 0), new Quaternion(0, 0, 0, 1)), new Double[] {
                    5e-1, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 5e-1, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 5e-1, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 5e-1, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 5e-1, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 5e-1
            }),
            new TwistWithCovariance(new Twist(new Vector3(0, 0, 0), new Vector3(0, 0, 0)), new Double[] {
                    1e-1, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1e-1, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1e-1, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1e-1, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1e-1, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1e-1
            }));

    public void publish() {
        double angularVelocity = Units
                .degreesToRadians(commandDriveTrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());
        Pose2d pose = commandDriveTrain.getState().Pose;
        if (DriveUtils.redAlliance()) {
            pose = DriveUtils.redBlueTransform(pose);
        }
        ChassisSpeeds velocity = commandDriveTrain.getChassisSpeeds();

        odomMsg.setHeader(odomPub.getHeader(Frames.ODOM_FRAME));
        odomMsg.getPose().setPose(ROSConversions.wpiToRosPose(new Pose3d(pose)));
        odomMsg.getTwist().getTwist()
                .setLinear(new Vector3(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0));
        odomMsg.getTwist().getTwist().setAngular(new Vector3(0.0, 0.0, angularVelocity));

        odomPub.send(odomMsg);
    }

}