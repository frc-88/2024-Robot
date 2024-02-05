package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ros.bridge.Frames;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;
import frc.robot.Constants;

public class Aiming extends SubsystemBase {
    private Pose2d relativePose;
    private Alliance alliance;
    private TFListenerCompact tf_compact;

    public Aiming() {

    }

    public void setTFListener(TFListenerCompact tfListener) {
        tf_compact = tfListener;
    }

    public Pose2d getROSPose() {
        Optional<Transform3dStamped> tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
        if (tfStamped.isEmpty()) {
            return new Pose2d();
        }
        return new Pose2d(tfStamped.get().transform.getTranslation().toTranslation2d(),
                tfStamped.get().transform.getRotation().toRotation2d());
    }

    public double speakerAngleForDrivetrian() {
        Pose2d robotPose = getROSPose();
        relativePose = (getAlliance() == DriverStation.Alliance.Red) ? robotPose.relativeTo(Constants.RED_SPEAKER_POSE)
                : robotPose.relativeTo(Constants.BLUE_SPEAKER_POSE);
        return Math.atan2(relativePose.getX(), relativePose.getY());
    }

    public double speakerAngleForShooter() {
        return 0;
    }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        return alliance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
