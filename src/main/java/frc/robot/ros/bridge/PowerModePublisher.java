package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosInt32;

public class PowerModePublisher implements Publisher {
    private final BridgePublisher<RosInt32> powerModePub;
    private int powerMode = 0;

    public PowerModePublisher(ROSNetworkTablesBridge bridge, int powerMode) {
        powerModePub = new BridgePublisher<>(bridge, "power_mode");
        this.powerMode = powerMode;
    }

    public void publish() {
        powerModePub.send(new RosInt32(this.powerMode));
    }
}