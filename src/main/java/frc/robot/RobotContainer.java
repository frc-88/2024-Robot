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
    SmartDashboard.putData("ClimberGoToPostition", m_climber.setPositionFactory());
    SmartDashboard.putData("ClimberGoToStart", m_climber.goToStartFactory());
    SmartDashboard.putData("ClimberCalibrate", m_climber.calibrateFactory());
    SmartDashboard.putData("ClimberCoastMode", m_climber.enableCoastModeFactory().ignoringDisable(true));
    SmartDashboard.putData("ClimberBrakeMode", m_climber.enableBrakeModeFactory().ignoringDisable(true));
    SmartDashboard.putData("ClimberUpDown", m_climber.upDownFactory());
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
