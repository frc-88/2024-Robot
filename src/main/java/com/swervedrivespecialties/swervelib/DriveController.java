package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix6.hardware.TalonFX;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistance();

    TalonFX getMotor();
}