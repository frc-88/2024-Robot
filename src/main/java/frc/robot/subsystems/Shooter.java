// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Shooter extends SubsystemBase {
    /** Creates a new Shooter. */

    private DoublePreferenceConstant leftShooterSpeed = new DoublePreferenceConstant("shooter/shooter/Leftspeed", 5600);
    private DoublePreferenceConstant rightShooterSpeed = new DoublePreferenceConstant("shooter/shooter/Rightspeed",
            5000);
    private DoublePreferenceConstant idleShooterControl = new DoublePreferenceConstant("shooter/shooter/idleControl",
            1500);
    private DoublePreferenceConstant p_slowSpeed = new DoublePreferenceConstant("shooter/shooter/slowspeed", 500);
    private DoublePreferenceConstant p_ampTrapSpeed = new DoublePreferenceConstant("shooter/shooter/AmpTrap", 2000);
    private DoublePreferenceConstant motor_kP = new DoublePreferenceConstant("shooter/shooter/motor_kP", 0.02);
    private DoublePreferenceConstant motor_kI = new DoublePreferenceConstant("shooter/shooter/motor_kI", 0);
    private DoublePreferenceConstant motor_kD = new DoublePreferenceConstant("shooter/shooter/motor_kD", 0);
    private DoublePreferenceConstant motor_kV = new DoublePreferenceConstant("shooter/shooter/motor_kV", 0.129);
    private DoublePreferenceConstant motor_kS = new DoublePreferenceConstant("shooter/shooter/motor_kS", 0);
    private DoublePreferenceConstant p_shuttlePassSpeed = new DoublePreferenceConstant(
            "shooter/shooter/ShuttlePassSpeed", 2750);
    private DoublePreferenceConstant p_sourceIntakeSpeed = new DoublePreferenceConstant(
            "shooter/shooter/SourceIntakeSpeed", 2800);
    private DoublePreferenceConstant p_shuttlePassSlowSpeed = new DoublePreferenceConstant("shooter/shooter/ShuttlePassSlowSpeed", 2500);

    private final TalonFX m_LeftShooter = new TalonFX(Constants.SHOOTER_LEFT_MOTOR, Constants.RIO_CANBUS);
    private final TalonFX m_RightShooter = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR, Constants.RIO_CANBUS);
    private double talonFree = 6380;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    // amplified and loud
    // a symphony of scoring
    // TJ rocks the house

    public Shooter() {
        applyAllConfigs(0);
        motor_kP.addChangeHandler(this::applyAllConfigs);
        motor_kI.addChangeHandler(this::applyAllConfigs);
        motor_kD.addChangeHandler(this::applyAllConfigs);
        motor_kV.addChangeHandler(this::applyAllConfigs);
        motor_kS.addChangeHandler(this::applyAllConfigs);
    }

    private void applyAllConfigs(double unused) {
        applyConfigs(m_LeftShooter);
        applyConfigs(m_RightShooter);
        m_LeftShooter.setInverted(true);
        m_RightShooter.setInverted(false);
    }

    private void applyConfigs(TalonFX motor) {

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = motor_kP.getValue(); // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = motor_kI.getValue(); // An error of 1 rotation per second increases output by 0.5V every
                                                // second
        configs.Slot0.kD = motor_kD.getValue(); // A change of 1 rotation per second squared results in 0.01 volts
                                                // output
        configs.Slot0.kV = motor_kV.getValue(); // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 =
                                                // 0.12
        // volts / Rotation per second
        configs.Slot0.kS = motor_kS.getValue();

        configs.CurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

    }

    public boolean isShooterAtFullSpeed() {
        return (Math.abs(m_LeftShooter.getVelocity().getValueAsDouble() * 60 - leftShooterSpeed.getValue()) <= 500
                && Math.abs(
                        m_RightShooter.getVelocity().getValueAsDouble() * 60 - rightShooterSpeed.getValue()) <= 500);
    }

    public boolean isShooterAtAmpTrapSpeed() {
        return (Math.abs(m_LeftShooter.getVelocity().getValueAsDouble() * 60 - p_ampTrapSpeed.getValue()) <= 500
                && Math.abs(
                        m_RightShooter.getVelocity().getValueAsDouble() * 60 - p_ampTrapSpeed.getValue()) <= 500);
    }

    public void startShooter() {
        m_LeftShooter.setControl(velocityRequest.withVelocity(leftShooterSpeed.getValue() / 60));
        m_RightShooter.setControl(velocityRequest.withVelocity(rightShooterSpeed.getValue() / 60));
    }

    public void stopShooter() {
        m_LeftShooter.stopMotor();
        m_RightShooter.stopMotor();
    }

    public void slowSpeed() {
        m_LeftShooter.setControl(velocityRequest.withVelocity(p_slowSpeed.getValue() / 60));
        m_RightShooter.setControl(velocityRequest.withVelocity(p_slowSpeed.getValue() / 60));
    }

    public void runIdleSpeed() {
        m_LeftShooter.setControl(velocityRequest.withVelocity(idleShooterControl.getValue() / 60));
        m_RightShooter.setControl(velocityRequest.withVelocity(idleShooterControl.getValue() / 60));
    }

    public void runAmpTrapSpeed() {
        m_LeftShooter.setControl(velocityRequest.withVelocity(p_ampTrapSpeed.getValue() / 60));
        m_RightShooter.setControl(velocityRequest.withVelocity(p_ampTrapSpeed.getValue() / 60));
    }

    public void runShuttlePassSpeed(boolean highSpeed) {
        m_LeftShooter.setControl(velocityRequest.withVelocity(highSpeed ? (p_shuttlePassSpeed.getValue() / 60) : (p_shuttlePassSlowSpeed.getValue() / 60)));
        m_RightShooter.setControl(velocityRequest.withVelocity(highSpeed ? (p_shuttlePassSpeed.getValue() / 60) : (p_shuttlePassSlowSpeed.getValue() / 60)));
    }

    public void runSourceIntake() {
        m_LeftShooter.setControl(velocityRequest.withVelocity(-p_sourceIntakeSpeed.getValue() / 60));
        m_RightShooter.setControl(velocityRequest.withVelocity(-p_sourceIntakeSpeed.getValue() / 60));
    }

    public Trigger shooterAtSpeed() {
        return new Trigger(() -> isShooterAtFullSpeed());
    }

    public Command runAmpTrapSpeedFactory() {
        return new RunCommand(() -> runAmpTrapSpeed(), this);
    }

    public Command runShooterFactory() {
        return new RunCommand(() -> startShooter(), this);
    }

    public Command stopShooterFactory() {
        return new RunCommand(() -> stopShooter(), this);
    }

    public Command runIdleSpeedFactory() {
        return new RunCommand(() -> runIdleSpeed(), this);
    }

    public Command runShuttlePassFactory(BooleanSupplier sourceSpeed) {
        return new RunCommand(() -> runShuttlePassSpeed(sourceSpeed.getAsBoolean()), this);
    }

    public Command slowSpeedFactory() {
        return new RunCommand(() -> slowSpeed(), this);
    }

    public Command runSourceIntakeFactory() {
        return new RunCommand(() -> runSourceIntake(), this);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left Shooter Speed", m_LeftShooter.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Shooter Speed", m_RightShooter.getVelocity().getValueAsDouble() * 60);
    }
}
