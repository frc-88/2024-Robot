package frc.robot.util;

import java.util.function.Consumer;

import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

/**
 * Class used for MotionMagic control, using Velocity, Acceleration, and Jerk
 */

public class MotionMagicPreferenceConstants {

    private DoublePreferenceConstant maxVelocity;
    private DoublePreferenceConstant maxAcceleration;
    private DoublePreferenceConstant maxJerk;

    /**
     * Consturctor for MotionMagic PreferenceConstants
     * 
     * @param name
     *                        The name of the Preference Constants
     * @param maxVelocity
     *                        The Max Velocity the Motion Profile can reach. Units
     *                        should be rotations per second
     * @param maxAcceleration
     *                        The Max Acceleration the Motion Profile can reach.
     *                        Units should be rotations per second squared
     * @param maxJerk
     *                        The Max Jerk the Motion Profile can reach. Units
     *                        should be rotations per second cubed
     */

    public MotionMagicPreferenceConstants(String name, double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = new DoublePreferenceConstant(String.format("%s MaxVelocity", name), maxVelocity);
        this.maxAcceleration = new DoublePreferenceConstant(String.format("%s MaxAcceleration", name), maxAcceleration);
        this.maxJerk = new DoublePreferenceConstant(String.format("%s MaxJerk", name), maxJerk);
    }

    public MotionMagicPreferenceConstants(String name) {
        this(name, 0, 0, 0);
    }

    /**
     * Get the MaxVelocity PreferenceConstant
     * 
     * @return The MaxVelocity PreferenceConstant
     */

    public DoublePreferenceConstant getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * Get the MaxAcceleration PreferenceConstant
     * 
     * @return The MaxAcceleration PreferenceConstant
     */

    public DoublePreferenceConstant getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Get the MaxJerk PreferenceConstant
     * 
     * @return The MaxJerk PreferenceConstant
     */

    public DoublePreferenceConstant getMaxJerk() {
        return maxJerk;
    }

    /**
     * Update Velocity, Acceleration, and Jerk PreferenceConstants
     * 
     */

    public void updateAll() {
        this.maxVelocity.update();
        this.maxAcceleration.update();
        this.maxJerk.update();
    }

    /**
     * Add the Consumer responsible taking in the preference and updating
     * 
     * @param handler The Consumer for updating the constant
     */

    public void addChangeHandler(Consumer<Double> handler) {
        this.maxVelocity.addChangeHandler(handler);
        this.maxAcceleration.addChangeHandler(handler);
        this.maxJerk.addChangeHandler(handler);
    }
}
