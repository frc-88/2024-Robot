// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Shooter m_shooter = new Shooter();
  public RobotContainer() {
    configureBindings();
    configureSmartDashboardButtons();
  }

private void configureSmartDashboardButtons() {
  //Shooter
  SmartDashboard.putData("Run Shooter", m_shooter.runShooterCommand());
  SmartDashboard.putData("Run Indexer", m_shooter.runIndexerCommand());
  SmartDashboard.putData("Stop Shooter", m_shooter.stopShooterCommand());
  SmartDashboard.putData("Stop Indexer", m_shooter.stopIndexerCommand());
}

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
