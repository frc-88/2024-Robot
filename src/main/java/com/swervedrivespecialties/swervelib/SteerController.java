package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix6.hardware.TalonFX;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void checkAngleReset();

    TalonFX getMotor();

    AbsoluteEncoder getEncoder();
}