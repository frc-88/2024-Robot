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



private DoublePreferenceConstant leftShooterSpeed =
  new DoublePreferenceConstant("shooter/shooter/Leftspeed", 0);
private DoublePreferenceConstant rightShooterSpeed =
  new DoublePreferenceConstant("shooter/shooter/Rightspeed", 0);

  final TalonFX m_LeftShooter = new TalonFX(8);
  final TalonFX m_RightShooter = new TalonFX(7);
  private double talonFree = 6380;

      TalonFXConfiguration configs = new TalonFXConfiguration();

  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

    m_robotContainer = new RobotContainer();


    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output


    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        StatusCode status_left = StatusCode.StatusCodeNotInitialized;
         StatusCode status_right = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status_left = m_LeftShooter.getConfigurator().apply(configs);
       status_right = m_RightShooter.getConfigurator().apply(configs);
      if (status_left.isOK() && status_right.isOK()) break;
    }
    if(!status_left.isOK() || !status_right.isOK()) {
      System.out.println("Could not apply configs, error code: Left: " + status_left.toString() + "Right: " + status_right.toString());
    }

    m_LeftShooter.setControl(new Follower(m_RightShooter.getDeviceID(), false));
  



  

  public Shooter (){
    m_LeftShooter.setInverted(true);
    m_RightShooter.setInverted(false);
  }

  public void startShooter() {
    m_LeftShooter.set(leftShooterSpeed.getValue() / talonFree);
    m_RightShooter.set( rightShooterSpeed.getValue() / talonFree);


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
   
    SmartDashboard.putNumber("Left Shooter Speed", m_LeftShooter.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("Right Shooter Speed", m_RightShooter.getVelocity().getValueAsDouble()*60);
  }
}
