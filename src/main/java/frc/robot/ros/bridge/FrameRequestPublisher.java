package frc.robot.ros.bridge;

import frc.robot.ros.messages.tj2_interfaces.RequestFrames;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.messages.DurationPrimitive;

public class FrameRequestPublisher implements Publisher {
    private final BridgePublisher<RequestFrames> requestPub;
    private double requestDuration = 0.0;

    public FrameRequestPublisher(ROSNetworkTablesBridge bridge, double requestDuration) {
        requestPub = new BridgePublisher<>(bridge, "request_frames");
        this.requestDuration = requestDuration;
    }

    public void setRequestDuration(double requestDuration) {
        this.requestDuration = requestDuration;
    }

    public void publish() {
        requestPub.send(new RequestFrames(requestPub.getHeader(""), new DurationPrimitive(requestDuration), 0));
    }
}
