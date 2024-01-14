// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_armSpeed = new DoublePreferenceConstant("Arm/Speed", -0.5);
  private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Arm/MotionMagicVelocity", 62.5);
  private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant("Arm/MotionMagicAcceleration", 250);
  private DoublePreferenceConstant p_maxJerk = new DoublePreferenceConstant("Arm/MotionMagicJerk", 500);
  private DoublePreferenceConstant p_targetRightPosition = new DoublePreferenceConstant("Arm/RightTarget", 0);
  private DoublePreferenceConstant p_targetLeftPosition = new DoublePreferenceConstant("Arm/LeftTarget", 0);
  private PIDPreferenceConstants p_PidPreferenceConstants = new PIDPreferenceConstants("Arm/PID");
  private final TalonFX m_armRight = new TalonFX(11);
  private final TalonFX m_armLeft = new TalonFX(9);

  private final DutyCycleOut m_armRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_armFollowerRequest = new DutyCycleOut(0.0);
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
  private double rightStartPosition;
  private double leftStartPosition;

  /** Creates a new Climber. */
  public Climber() {
    configureTalons(m_armRight);
    configureTalons(m_armLeft);
    m_armRight.setNeutralMode(NeutralModeValue.Brake);
    m_armLeft.setNeutralMode(NeutralModeValue.Brake);
    rightStartPosition = m_armRight.getPosition().getValueAsDouble();
    leftStartPosition = m_armLeft.getPosition().getValueAsDouble();
  }

  private void configureTalons(TalonFX talon) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = p_maxVelocity.getValue();
    mm.MotionMagicAcceleration = p_maxAcceleration.getValue();
    mm.MotionMagicJerk = p_maxJerk.getValue();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = p_PidPreferenceConstants.getKP().getValue();
    slot0.kI = p_PidPreferenceConstants.getKI().getValue();
    slot0.kD = p_PidPreferenceConstants.getKD().getValue();
    slot0.kV = p_PidPreferenceConstants.getKF().getValue();
    slot0.kS = p_PidPreferenceConstants.getKS().getValue(); // Approximately 0.25V to get the mechanism moving

    talon.getConfigurator().apply(cfg);
  }

  public void setPostion(double rightPosition, double leftPosition) {
    m_armRight.setControl(m_motionMagic.withPosition(rightPosition));
    m_armLeft.setControl(m_motionMagic.withPosition(leftPosition));
  }

  public void set(double speed) {
    m_armRight.setControl(m_armRequest.withOutput(speed));
    m_armLeft.setControl(m_armFollowerRequest.withOutput(speed));
  }

  public void run() {
    set(p_armSpeed.getValue());
  }

  public void stop() {
    set(0);
  }

  public Command runFactory() {
    return new RunCommand(() -> {run();}, this);
  }

  public Command stopFactory() {
    return new InstantCommand(() -> {stop();}, this);
  }

  public Command setPositionFactory() {
    return new RunCommand(() -> {setPostion(p_targetRightPosition.getValue(), p_targetLeftPosition.getValue());}, this);
  }

  public Command goToStartFactory() {
    return new RunCommand(() -> {setPostion(rightStartPosition, leftStartPosition);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber:ArmRight", m_armRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber:ArmLeft", m_armLeft.getPosition().getValueAsDouble());
  }
}
