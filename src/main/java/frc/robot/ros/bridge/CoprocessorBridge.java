// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ros.bridge;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;

public class CoprocessorBridge extends SubsystemBase {
    private final TFListenerCompact tfListenerCompact;
    private final PingPublisher pingPublisher;
    private final PowerModePublisher powerModePublisher;
    private final PreferenceBackupPublisher preferenceBackupPublisher;
    private final OdomPublisher odomPublisher;

    private final Publisher[] periodicPublishers;

    private final ROSNetworkTablesBridge bridge;
    private boolean coprocessorAlive = false;
    private Timer timer = new Timer();
    private int counter = 0;

    public CoprocessorBridge(CommandSwerveDrivetrain drive, ROSNetworkTablesBridge bridge,
            TFListenerCompact tfListenerCompact) {
        this.bridge = bridge;
        this.tfListenerCompact = tfListenerCompact;
        this.pingPublisher = new PingPublisher(bridge);
        this.powerModePublisher = new PowerModePublisher(bridge, PowerModePublisher.JetsonPowerMode.HIGH_POWER);
        this.preferenceBackupPublisher = new PreferenceBackupPublisher(bridge);
        odomPublisher = new OdomPublisher(drive, bridge);
        periodicPublishers = new Publisher[] { pingPublisher, odomPublisher };
    }

    public void onCoprocessorAlive() {
        preferenceBackupPublisher.publish();
        powerModePublisher.publish();
        timer.start();
        counter = 0;
    }

    // ---
    // Periodics
    // ---

    @Override
    public void periodic() {

        if (bridge.isAlive() && !coprocessorAlive) {
            coprocessorAlive = true;
            onCoprocessorAlive();
        }
        tfListenerCompact.update();
        for (Publisher publisher : periodicPublishers) {
            publisher.publish();
        }
        if (coprocessorAlive) {
            SmartDashboard.putNumber("ROS avg cycle time", timer.get() / ++counter);
        }
    }
}