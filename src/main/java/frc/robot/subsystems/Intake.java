package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private DoublePreferenceConstant intakeRollerSpeed = new DoublePreferenceConstant("Intake/IntakeRollerSpeed", 1);
    private DoublePreferenceConstant guideRollerSpeed = new DoublePreferenceConstant("Intake/GuideRollerSpeed", 1);
    private DoublePreferenceConstant indexRollerSpeed = new DoublePreferenceConstant("Intake/IndexRollerSpeed", 0.25);
    private DoublePreferenceConstant indexShootSpeed = new DoublePreferenceConstant("Intake/IndexerShootSetup", 1);
    private DoublePreferenceConstant p_indexSourceSpeed = new DoublePreferenceConstant("Intake/IndexSourceSpeed", 0.4);

    private DoublePreferenceConstant p_debounceTime = new DoublePreferenceConstant("Intake/IndexDebounceTime", 0.25);

    private final DutyCycleOut m_intakeRequest = new DutyCycleOut(0.0);
    private final TalonFX m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, Constants.RIO_CANBUS);
    private final TalonFX m_guideMotor = new TalonFX(Constants.INTAKE_GUIDE_MOTOR_ID, Constants.RIO_CANBUS);
    private final TalonFX m_indexMotor = new TalonFX(Constants.INTAKE_INDEX_MOTOR_ID, Constants.RIO_CANBUS);
    private BooleanSupplier m_elevatorAndPivotDown;

    private final TalonFXConfiguration indexConfiguration = new TalonFXConfiguration();

    private Trigger m_hasNoteDebounced = new Trigger(this::hasNoteInIndexer).debounce(p_debounceTime.getValue(),
            DebounceType.kBoth);

    public boolean m_automaticMode = true;
    public boolean lastMode = true;

    // I heard some folks say
    // my recipes are one note
    // but this intake cooks

    public Intake(BooleanSupplier elevatorAndPivotDown) {
        m_elevatorAndPivotDown = elevatorAndPivotDown;
        configureTalons();
    }

    public void configureTalons() {
        // There are many configs we can set
        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
        intakeConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_CURRENT_LIMIT;
        intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfiguration.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(.5);
        m_intakeMotor.getConfigurator().apply(intakeConfiguration);
        m_intakeMotor.setInverted(true);

        TalonFXConfiguration guideConfiguration = new TalonFXConfiguration();
        guideConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_CURRENT_LIMIT;
        guideConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        guideConfiguration.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(.5);
        m_guideMotor.getConfigurator().apply(guideConfiguration);

        indexConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_CURRENT_LIMIT;
        indexConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexConfiguration.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(.5);
        m_indexMotor.getConfigurator().apply(indexConfiguration);
        m_indexMotor.setInverted(true);
    }

    public void intake() {
        if (m_elevatorAndPivotDown.getAsBoolean()) {
            m_indexMotor.getConfigurator().refresh(indexConfiguration);
            indexConfiguration.HardwareLimitSwitch.ForwardLimitEnable = true;
            m_indexMotor.getConfigurator().apply(indexConfiguration);
            m_indexMotor.setInverted(true);
            m_intakeMotor.setControl(m_intakeRequest.withOutput(intakeRollerSpeed.getValue()));
            m_guideMotor.setControl(m_intakeRequest.withOutput(guideRollerSpeed.getValue()));
            m_indexMotor.setControl(m_intakeRequest.withOutput(indexRollerSpeed.getValue()));
        } else {
            stopMoving();
        }
    }

    public void stopMoving() {
        m_intakeMotor.setControl(m_intakeRequest.withOutput(0));
        m_guideMotor.setControl(m_intakeRequest.withOutput(0));
        m_indexMotor.setControl(m_intakeRequest.withOutput(0));
    }

    public void shootIndexer() {
        m_indexMotor.getConfigurator().refresh(indexConfiguration);
        indexConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        m_indexMotor.getConfigurator().apply(indexConfiguration);
        m_indexMotor.setInverted(true);
        m_indexMotor.setControl(m_intakeRequest.withOutput(indexShootSpeed.getValue()));
        m_intakeMotor.setControl(m_intakeRequest.withOutput(0));
        m_guideMotor.setControl(m_intakeRequest.withOutput(0));
    }

    public void reject() {
        m_intakeMotor.setControl(m_intakeRequest.withOutput(-1));
        m_guideMotor.setControl(m_intakeRequest.withOutput(-1));
        m_indexMotor.setControl(m_intakeRequest.withOutput(-1));
    }

    public void sourceIntake() {
        m_intakeMotor.stopMotor();
        m_guideMotor.stopMotor();
        m_indexMotor.setControl(m_intakeRequest.withOutput(-p_indexSourceSpeed.getValue()));
    }

    public void goblinMode() {
        m_indexMotor.getConfigurator().refresh(indexConfiguration);
        indexConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        m_indexMotor.getConfigurator().apply(indexConfiguration);
        m_intakeMotor.setControl(m_intakeRequest.withOutput(intakeRollerSpeed.getValue()));
        m_guideMotor.setControl(m_intakeRequest.withOutput(guideRollerSpeed.getValue()));
        m_indexMotor.setControl(m_intakeRequest.withOutput(indexShootSpeed.getValue()));
    }

    public boolean isIntakeReady() {
        return m_intakeMotor.getMotorVoltage().getValueAsDouble() > 6.0
                && m_guideMotor.getMotorVoltage().getValueAsDouble() > 6.0;
    }

    public boolean isIndexerReady() {
        return m_indexMotor.getMotorVoltage().getValueAsDouble() > 6.0;
    }

    public Command intakeFactory() {
        return new RunCommand(() -> intake(), this).until(() -> hasNoteInIndexer());
    }

    public Command goblinModeFactory() {
        return new RunCommand(() -> goblinMode(), this);
    }

    public Command rejectFactory() {
        return new RunCommand(() -> reject(), this);
    }

    public Command stopMovingFactory() {
        return new RunCommand(() -> stopMoving(), this);
    }

    public Command shootIndexerFactory() {
        return new RunCommand(() -> shootIndexer(), this);
    }

    public Command sourceIntakeFactory() {
        return new RunCommand(() -> sourceIntake(), this);
    }

    public boolean hasNoteInIndexer() {
        return m_indexMotor.getForwardLimit().getValueAsDouble() == 0;
    }

    public void disableAutoMode() {
        lastMode = hasNote().getAsBoolean();
        m_automaticMode = false;
    }

    public void enableAutoMode() {
        m_automaticMode = true;
    }

    public Trigger hasNote() {
        return new Trigger(() -> (m_automaticMode) ? hasNoteInIndexer() : lastMode);
    }

    public Trigger hasNoteDebounced() {
        return m_hasNoteDebounced;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Intake Speed", m_intakeMotor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Intake/Guide Speed", m_guideMotor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Intake/Index Speed", m_indexMotor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putBoolean("Intake/Index Has Note", hasNoteInIndexer());
        SmartDashboard.putNumber("Intake/Intake Current", m_intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Guide Current", m_guideMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Index Current", m_indexMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/HasNoteDebounced", hasNoteDebounced().getAsBoolean());
    }
}
