package frc.robot.ros.bridge;

import java.util.Optional;

import frc.robot.ros.messages.tj2_interfaces.GameObjectsStamped;
import frc.team88.ros.bridge.BridgeSubscriber;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;

public class GameObjectsSubscriber implements Subscriber<GameObjectsStamped> {
    private final BridgeSubscriber<GameObjectsStamped> objectsSub;
    private GameObjectsStamped lastObjects = new GameObjectsStamped();

    public GameObjectsSubscriber(ROSNetworkTablesBridge bridge) {
        objectsSub = new BridgeSubscriber<>(bridge, "detections", GameObjectsStamped.class);
    }

    public Optional<GameObjectsStamped> receive() {
        Optional<GameObjectsStamped> optMsg;
        if ((optMsg = objectsSub.receive()).isPresent()) {
            lastObjects = optMsg.get();
            return optMsg;
        }
        return optMsg;
    }

    public GameObjectsStamped getLastObjects() {
        receive();
        return lastObjects;
    }
}