package frc.robot.ros.bridge;

import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.std_msgs.RosInt32;

public class PowerModePublisher implements Publisher {
    public enum JetsonPowerMode {
        UNKNOWN(-1),
        LOW_POWER(3),
        HIGH_POWER(6);

        public final int value;

        private JetsonPowerMode(int value) {
            this.value = value;
        }
    }

    private final BridgePublisher<RosInt32> powerModePub;
    private JetsonPowerMode powerMode = JetsonPowerMode.UNKNOWN;

    public PowerModePublisher(ROSNetworkTablesBridge bridge, JetsonPowerMode powerMode) {
        powerModePub = new BridgePublisher<>(bridge, "power_mode");
        this.powerMode = powerMode;
    }

    public void publish() {
        powerModePub.send(new RosInt32(this.powerMode.value));
    }
}