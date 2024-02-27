// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
    private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Arm/MotionMagicVelocity", 62.5);
    private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant("Arm/MotionMagicAcceleration",
            250);
    private DoublePreferenceConstant p_maxJerk = new DoublePreferenceConstant("Arm/MotionMagicJerk", 500);
    private DoublePreferenceConstant p_armTarget = new DoublePreferenceConstant("Arm/ArmTarget", 0);
    private DoublePreferenceConstant p_armStowSpeed = new DoublePreferenceConstant("Arm/ArmStowSpeed", 0);
    private DoublePreferenceConstant p_armPrepPosition = new DoublePreferenceConstant("Arm/ArmPrepPosition", -20);
    private DoublePreferenceConstant p_armClimbPosition = new DoublePreferenceConstant("Arm/ArmClimbPosition", 0);
    private PIDPreferenceConstants p_PidPreferenceConstants = new PIDPreferenceConstants("Arm/PID");
    private final TalonFX m_armRight = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR, Constants.RIO_CANBUS);
    private final TalonFX m_armLeft = new TalonFX(Constants.CLIMBER_LEFT_MOTOR, Constants.RIO_CANBUS);

    private final Debouncer climberDebouncer = new Debouncer(1);

    private final DutyCycleOut m_armRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut m_armFollowerRequest = new DutyCycleOut(0.0);
    // create a Motion Magic request, voltage output
    private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
    private double rightStartPosition;
    private double leftStartPosition;
    private final double kMotorRotationsToClimberPosition = 360.0 / 250.0;
    private double m_angle;

    /** Creates a new Climber. */
    public Climber() {
        configureTalons(0);
        m_armRight.setNeutralMode(NeutralModeValue.Brake);
        m_armLeft.setNeutralMode(NeutralModeValue.Brake);
        rightStartPosition = m_armRight.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition;
        leftStartPosition = m_armLeft.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition;

        p_maxVelocity.addChangeHandler(this::configureTalons);
        p_maxAcceleration.addChangeHandler(this::configureTalons);
        p_maxJerk.addChangeHandler(this::configureTalons);
        p_PidPreferenceConstants.addChangeHandler(this::configureTalons);
    }

    private void configureTalonsSlow(double unused) {
        configureTalons(unused, p_maxVelocity.getValue() / 4.0);
    }

    private void configureTalons(double unused) {
        configureTalons(unused, p_maxVelocity.getValue());
    }

    private void configureTalons(double unused, double velocity) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = velocity;
        mm.MotionMagicAcceleration = p_maxAcceleration.getValue();
        mm.MotionMagicJerk = p_maxJerk.getValue();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = p_PidPreferenceConstants.getKP().getValue();
        slot0.kI = p_PidPreferenceConstants.getKI().getValue();
        slot0.kD = p_PidPreferenceConstants.getKD().getValue();
        slot0.kV = p_PidPreferenceConstants.getKF().getValue();
        slot0.kS = p_PidPreferenceConstants.getKS().getValue(); // Approximately 0.25V to get the mechanism moving

        m_armLeft.getConfigurator().apply(cfg);
        m_armRight.getConfigurator().apply(cfg);

        m_armLeft.setInverted(true);
    }

    public boolean climberOnTarget(double position, double tolerance) {
        return Math.abs(
                m_armLeft.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition - position) < tolerance
                && Math.abs(
                        m_armRight.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition
                                - position) < tolerance;
    }

    public void stowArms() {
        m_armRight.setControl(new DutyCycleOut(-p_armStowSpeed.getValue()));
        m_armLeft.setControl(new DutyCycleOut(-p_armStowSpeed.getValue()));
        if (climberDebouncer.calculate(m_armRight.getVelocity().getValueAsDouble() > -1)
                && climberDebouncer.calculate(m_armLeft.getVelocity().getValueAsDouble() > -1)) {
            calibrate();
        }
    }

    public void softLanding() {
        m_armRight.setControl(new DutyCycleOut(-p_armStowSpeed.getValue()));
        m_armLeft.setControl(new DutyCycleOut(-p_armStowSpeed.getValue()));
    }

    public void setClimberPostion(double position) {
        m_angle = position;
        m_armRight.setControl(m_motionMagic.withPosition(position / kMotorRotationsToClimberPosition));
        m_armLeft.setControl(m_motionMagic.withPosition(position / kMotorRotationsToClimberPosition));
    }

    public void setClimberPostion(DoubleSupplier position) {
        m_armRight.setControl(m_motionMagic.withPosition(position.getAsDouble() / kMotorRotationsToClimberPosition));
        m_armLeft.setControl(m_motionMagic.withPosition(position.getAsDouble() / kMotorRotationsToClimberPosition));
    }

    public void set(double speed) {
        m_armRight.setControl(m_armRequest.withOutput(speed));
        m_armLeft.setControl(m_armFollowerRequest.withOutput(speed));
    }

    public void calibrate() {
        // rightStartPosition = m_armRight.getPosition().getValueAsDouble() *
        // kMotorRotationsToClimberPosition;
        // leftStartPosition = m_armLeft.getPosition().getValueAsDouble() *
        // kMotorRotationsToClimberPosition;
        m_armRight.setPosition(-79.0 / kMotorRotationsToClimberPosition);
        m_armLeft.setPosition(-79.0 / kMotorRotationsToClimberPosition);
    }

    public void enableCoastMode() {
        m_armLeft.setNeutralMode(NeutralModeValue.Coast);
        m_armRight.setNeutralMode(NeutralModeValue.Coast);
    }

    public void enableBrakeMode() {
        m_armLeft.setNeutralMode(NeutralModeValue.Brake);
        m_armRight.setNeutralMode(NeutralModeValue.Brake);
    }

    public void addToOrchestra(Orchestra m_orchestra) {
        m_orchestra.addInstrument(m_armLeft);
        m_orchestra.addInstrument(m_armRight);
    }

    // public Command softLanding() {
    // return new InstantCommand(() -> configureTalonsSlow(0)).andThen(new
    // WaitCommand(0.25))
    // .andThen(new RunCommand(() ->
    // setClimberPostion(p_armPrepPosition.getValue()), this))
    // .until(this::onTarget).finallyDo(() -> configureTalons(0));
    // }

    public Command softLandingFactory() {
        return new RunCommand(() -> softLanding(), this)
                .until(() -> climberOnTarget(p_armPrepPosition.getValue(), 2.0));
    }

    public Command climbFactory() {
        return new RunCommand(() -> setClimberPostion(p_armClimbPosition.getValue()), this);
    }

    public Command prepArmsFactory() {
        return new RunCommand(() -> setClimberPostion(p_armPrepPosition.getValue()), this);
    }

    public Command stowArmFactory() {
        return new RunCommand(() -> stowArms(), this).beforeStarting(() -> climberDebouncer.calculate(false));
    }

    public Command calibrateFactory() {
        return new InstantCommand(() -> {
            calibrate();
        }, this);
    }

    public Command enableCoastModeFactory() {
        return new InstantCommand(() -> {
            enableCoastMode();
        }, this);
    }

    public Command enableBrakeModeFactory() {
        return new InstantCommand(() -> {
            enableBrakeMode();
        }, this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber:ArmRight",
                m_armRight.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition);
        SmartDashboard.putNumber("Climber:ArmLeft",
                m_armLeft.getPosition().getValueAsDouble() * kMotorRotationsToClimberPosition);
        SmartDashboard.putNumber("Climber:StartLeft", leftStartPosition);
        SmartDashboard.putNumber("Climber:StartRight", rightStartPosition);
    }
}
