// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoprocessorBridge;

import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

public class RobotContainer {
    private TFListenerCompact tfListenerCompact;
    @SuppressWarnings("unused")
    private CoprocessorBridge coprocessorBridge;

    public RobotContainer() {
        configureBindings();
        configureRosNetworkTablesBridge();
    }

    private void configureBindings() {
    }

    public void configureRosNetworkTablesBridge() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        ROSNetworkTablesBridge bridge = new ROSNetworkTablesBridge(instance.getTable(""), 0.02);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        coprocessorBridge = new CoprocessorBridge(tfListenerCompact);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
