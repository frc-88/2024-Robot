// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_armSpeed = new DoublePreferenceConstant("Arm/Speed", -0.5);
  
  private final TalonFX m_armLeader = new TalonFX(0, "rio");
  private final TalonFX m_armFollower = new TalonFX(1, "rio");
  
  private final DutyCycleOut m_armRequest =  new DutyCycleOut(0.0);
  private final DutyCycleOut m_armFollowerRequest = new DutyCycleOut(0.0);
  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  /** Creates a new Climber. */
  public Climber() {


  }

  private void configureTalons(TalonFX talon) {
     TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 62.5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 250; // Take approximately 0.25 seconds to reach max vel
    mm.MotionMagicJerk = 500;  // Take approximately 0.5 seconds to reach max accel 

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    talon.getConfigurator().apply(cfg);
  }

  public void setPostion(double position){
    m_armLeader.setControl(m_motionMagic.withPosition(position));
  }

  public void set(double speed){
    m_armLeader.setControl(m_armRequest.withOutput(speed));
    m_armFollower.setControl(m_armFollowerRequest.withOutput(speed));
  }

  public void run(){
    set(p_armSpeed.getValue());
  }

  public void stop(){
    set(0);
  }

  public Command runFactory(){
    return new RunCommand(() -> {run();}, this);
  }

  public Command stopFactory(){
    return new InstantCommand(() -> {stop();}, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber:Arm:Postion", m_armLeader.getPosition().getValueAsDouble());
  }
}
