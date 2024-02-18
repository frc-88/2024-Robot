package frc.robot.util;

import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class MotionMagicConstants {

    private DoublePreferenceConstant maxVelocity;
    private DoublePreferenceConstant maxAcceleration;
    private DoublePreferenceConstant maxJerk;

    public MotionMagicConstants(String name, double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = new DoublePreferenceConstant(String.format("%s MaxVelocity", name), maxVelocity);
        this.maxAcceleration = new DoublePreferenceConstant(String.format("%s MaxVelocity", name), maxVelocity);
        this.maxJerk = new DoublePreferenceConstant(String.format("%s MaxVelocity", name), maxVelocity);
    }

    public MotionMagicConstants(String name) {
        this(name, 0, 0, 0);
    }

    public double getMaxVelocity() {
        return maxVelocity.getValue();
    }
}
