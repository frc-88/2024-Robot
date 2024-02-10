// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Shooter extends SubsystemBase {
    /** Creates a new Shooter. */

    private DoublePreferenceConstant leftShooterSpeed = new DoublePreferenceConstant("shooter/shooter/Leftspeed", 0);
    private DoublePreferenceConstant rightShooterSpeed = new DoublePreferenceConstant("shooter/shooter/Rightspeed", 0);

    final TalonFX m_LeftShooter = new TalonFX(8);
    final TalonFX m_RightShooter = new TalonFX(7);
    private double talonFree = 6380;

    public Shooter() {
        m_LeftShooter.setInverted(true);
        m_RightShooter.setInverted(false);
        applyConfigs(m_LeftShooter);
        applyConfigs(m_RightShooter);
    }

    public void applyConfigs(TalonFX motor) {

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

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

    public void startShooter() {
        m_LeftShooter.set(leftShooterSpeed.getValue() / talonFree);
        m_RightShooter.set(rightShooterSpeed.getValue() / talonFree);
    }

    public void stopShooter() {
        m_LeftShooter.set(0);
        m_RightShooter.set(0);
    }

    public Command runShooterCommand() {
        return new RunCommand(() -> {
            startShooter();
        }, this);
    }

    public Command stopShooterCommand() {
        return new RunCommand(() -> {
            stopShooter();
        }, this);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Left Shooter Speed", m_LeftShooter.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Shooter Speed", m_RightShooter.getVelocity().getValueAsDouble() * 60);
    }
}
