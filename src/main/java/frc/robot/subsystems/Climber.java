// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase { 
  private final TalonFX m_armLeader = new TalonFX(0, "rio");
  private final TalonFX m_armFollower = new TalonFX(1, "rio");
  
  private final DutyCycleOut m_armRequest =  new DutyCycleOut(0.0);
  private final DutyCycleOut m_armFollowerRequest = new DutyCycleOut(0.0);

  /** Creates a new Climber. */
  public Climber() {}

  public void set(double speed){
    m_armLeader.setControl(m_armRequest.withOutput(speed));
    m_armFollower.setControl(m_armFollowerRequest.withOutput(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
