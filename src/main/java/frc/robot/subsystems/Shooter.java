// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

private DoublePreferenceConstant leftShooterSpeed =
  new DoublePreferenceConstant("shooter/shooter/Leftspeed", 0);
private DoublePreferenceConstant rightShooterSpeed =
  new DoublePreferenceConstant("shooter/shooter/Rightspeed", 0);

  final TalonFX m_LeftShooter = new TalonFX(8);
  final TalonFX m_RightShooter = new TalonFX(7);
  

private double talonFree = 6380;

  public void startShooter() {
    m_LeftShooter.set(leftShooterSpeed.getValue()/talonFree);
    m_RightShooter.set(rightShooterSpeed.getValue()/talonFree);
    }  
 
  public void stopShooter() {
    m_LeftShooter.set(0);
    m_RightShooter.set(0);
    }
 
  public Command runShooterCommand(){
    return new RunCommand(() -> {startShooter();}, this);
  }
  
  public Command stopShooterCommand(){
    return new RunCommand(() -> {stopShooter();}, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
