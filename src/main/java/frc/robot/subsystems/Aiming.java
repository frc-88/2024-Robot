package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ros.bridge.Frames;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.conversions.Transform3dStamped;

public class Aiming extends SubsystemBase {
    private final Pose2d redSpeakerPose = new Pose2d(new Translation2d(16.579342, 5.547868), new Rotation2d(180));
    private final Pose2d blueSpeakerPose = new Pose2d(new Translation2d(-0.0381, 5.547868), new Rotation2d(0));

    private TFListenerCompact tf_compact;

    public Aiming() {

    }

    public void setTFListener(TFListenerCompact tfListener) {
        tf_compact = tfListener;
    }

    public Pose2d getROSPose() {
        Transform3dStamped tfStamped = tf_compact.lookupTransform(Frames.MAP_FRAME, Frames.BASE_FRAME);
        Translation2d XYTranslation = tfStamped.transform.getTranslation().toTranslation2d();
        Rotation2d rotation = tfStamped.transform.getRotation().toRotation2d();
        return new Pose2d(XYTranslation, rotation);
    }

    public double speakerAngleForDrivetrian() {
        return 0;
    }

    public double speakerAngleForShooter() {
        return 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
