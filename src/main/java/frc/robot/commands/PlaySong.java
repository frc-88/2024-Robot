package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PlaySong extends Command {
    String filename;
    Orchestra m_orchestra = new Orchestra();
    Intake m_intake;
    Shooter m_shooter;
    Climber m_climber;
    Elevator m_elevator;
    CommandSwerveDrivetrain m_swerve;

    public PlaySong(String filename, Intake intake, Shooter shooter, Climber climber, Elevator elevator,
            CommandSwerveDrivetrain swerve) {
        this.filename = filename;
        addRequirements(intake, shooter, climber, elevator, swerve);
        m_intake = intake;
        m_shooter = shooter;
        m_climber = climber;
        m_elevator = elevator;
        m_swerve = swerve;
    }

    public void initialize() {
        m_intake.addToOrchestra(m_orchestra);
        m_shooter.addToOrchestra(m_orchestra);
        m_climber.addToOrchestra(m_orchestra);
        m_elevator.addToOrchestra(m_orchestra);
        m_swerve.addToOrchestra(m_orchestra);
        m_orchestra.loadMusic(filename);
        m_orchestra.play();
    }

    public void end() {
        m_orchestra.stop();
        m_orchestra.clearInstruments();
    }

    public boolean isFinished() {
        return !m_orchestra.isPlaying();
    }
}
