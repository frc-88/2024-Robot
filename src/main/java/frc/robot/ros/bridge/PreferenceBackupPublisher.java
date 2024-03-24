package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosTime;

public class PreferenceBackupPublisher implements Publisher {
    private BridgePublisher<RosTime> preferenceBackupPub;

    public PreferenceBackupPublisher(ROSNetworkTablesBridge bridge) {
        preferenceBackupPub = new BridgePublisher<>(bridge, "backup_preferences");
    }

    public void publish() {
        preferenceBackupPub.send(new RosTime(preferenceBackupPub.getNow()));
    }
}
