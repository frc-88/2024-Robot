// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team88.ros.conversions.TFListenerCompact;

public class CoprocessorBridge extends SubsystemBase {
    private final TFListenerCompact tfListenerCompact;

    public CoprocessorBridge(TFListenerCompact tfListenerCompact) {
        this.tfListenerCompact = tfListenerCompact;
    }

    // ---
    // Periodics
    // ---

    @Override
    public void periodic() {
        tfListenerCompact.update();
    }
}