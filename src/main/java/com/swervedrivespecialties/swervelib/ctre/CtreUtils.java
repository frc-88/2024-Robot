package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
    private CtreUtils() {
    }

    public static void checkCtreError(StatusCode statusCode, String message) {
        if (!statusCode.isOK()) {
            DriverStation.reportError(String.format("%s: %s", message, statusCode.toString()), false);
        }
    }
}