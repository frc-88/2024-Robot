// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;

public class RobotContainer {
  private Climber m_climber = new Climber();


  public RobotContainer() {
    configureBindings();
    configureSmartDashboardButtons();
  } 


  private void configureBindings() {}

  private void configureSmartDashboardButtons(){
    SmartDashboard.putData("ClimberRun", m_climber.runFactory());
    SmartDashboard.putData("ClimberStop", m_climber.stopFactory());
    SmartDashboard.putData("ClimberGoToPostition", m_climber.setPositionFactory());
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
