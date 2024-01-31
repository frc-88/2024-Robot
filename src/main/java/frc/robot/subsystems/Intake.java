package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private DoublePreferenceConstant intakeRollerSpeed = new DoublePreferenceConstant("Intake/IntakeRollerSpeed", 0);
    private DoublePreferenceConstant guideRollerSpeed = new DoublePreferenceConstant("Intake/GuideRollerSpeed", 0);
    private DoublePreferenceConstant indexRollerSpeed = new DoublePreferenceConstant("Intake/IndexRollerSpeed", 0);

    private final DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);
    private final TalonFX m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, Constants.INTAKE_CANBUS);
    private final TalonFX m_guideMotor = new TalonFX(Constants.INTAKE_GUIDE_MOTOR_ID, Constants.INTAKE_CANBUS);
    private final TalonFX m_indexMotor = new TalonFX(Constants.INTAKE_INDEX_MOTOR_ID, Constants.INTAKE_CANBUS);

    public Intake() {
        configureTalons(m_intakeMotor, m_guideMotor, m_indexMotor);
    }

    public void configureTalons(TalonFX... talons) {
        // There are many configs we can set
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.CurrentLimits.SupplyCurrentLimit = 10.0;

        for (TalonFX motor : talons) {
            motor.getConfigurator().apply(configuration);
        }
    }

    public void intake() {
        m_intakeMotor.setControl(m_intakeRequest.withOutput(intakeRollerSpeed.getValue()));
        m_guideMotor.setControl(m_intakeRequest.withOutput(guideRollerSpeed.getValue()));
        m_indexMotor.setControl(m_intakeRequest.withOutput(indexRollerSpeed.getValue()));
    }

    public Command intakeFactory() {
        return new RunCommand(() -> intake(), this);
    }

    public boolean getSensorValue() {
        return m_indexMotor.getForwardLimit().getValueAsDouble() == 0;
    }

    public TalonFX getIndexerMotor() {
        return m_indexMotor;
    }

}
