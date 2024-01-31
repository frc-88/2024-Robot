package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
    private DoublePreferenceConstant innerRollerSpeed = new DoublePreferenceConstant("Intake/InnerRollerSpeed", 0);
    private DoublePreferenceConstant outerRollerSpeed = new DoublePreferenceConstant("Intake/OuterRollerSpeed", 0);

    // ir sensor?
    // private final AnalogInput m_irSensor;

    private final DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);

    private final TalonFX m_innerMotor = new TalonFX(0);
    private final TalonFX m_outerMotor = new TalonFX(1);

    public Intake() {
        configureTalons();
    }

    public void configureTalons() {
        // There are many configs we can set
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.CurrentLimits.SupplyCurrentLimit = 10.0;
    }

    public void intake() {
        m_innerMotor.setControl(m_intakeRequest.withOutput(innerRollerSpeed.getValue()));
        m_outerMotor.setControl(m_intakeRequest.withOutput(outerRollerSpeed.getValue()));
    }

    public Command intakeFactory() {
        return new RunCommand(() -> intake(), this);
    }

}
