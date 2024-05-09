package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SuppliedWaitCommand extends WaitCommand {

    private final DoubleSupplier m_durationSupplier;

    @SuppressWarnings("this-escape")
    public SuppliedWaitCommand(DoubleSupplier seconds) {
        super(seconds.getAsDouble());
        m_durationSupplier = seconds;
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_durationSupplier.getAsDouble());
    }
}
