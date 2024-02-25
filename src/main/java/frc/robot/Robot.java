// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PreferenceConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private DoublePreferenceConstant p_teleopTime = new DoublePreferenceConstant("Runtime/Teleop", 0.0);
    private DoublePreferenceConstant p_autoTime = new DoublePreferenceConstant("Runtime/Auto", 0.0);
    private Timer teleopTimer = new Timer();
    private Timer autoTimer = new Timer();

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        PreferenceConstants.update();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.disabledPeriodic();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autoTimer.reset();
        autoTimer.start();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        m_robotContainer.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        autoTimer.stop();
        p_autoTime.setValue(p_autoTime.getValue() + autoTimer.get());
    }

    @Override
    public void teleopInit() {
        teleopTimer.reset();
        teleopTimer.start();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        teleopTimer.stop();
        p_teleopTime.setValue(p_teleopTime.getValue() + teleopTimer.get());
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
