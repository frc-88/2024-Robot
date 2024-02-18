package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;
import frc.robot.util.preferenceconstants.PreferenceConstant;
import frc.robot.util.preferenceconstants.PreferenceConstants;

public class Elevator extends SubsystemBase {
    private DoublePreferenceConstant p_pivotPodium = new DoublePreferenceConstant("Elevator/Podium", 60);
    private DoublePreferenceConstant p_pivotFlat = new DoublePreferenceConstant("Elevator/Flat", 90);
    private DoublePreferenceConstant p_PivotMaxVelocity = new DoublePreferenceConstant(
            "Elevator/PivotMotionMagicVelocity", 0);
    private DoublePreferenceConstant p_PivotMaxAcceleration = new DoublePreferenceConstant(
            "Elevator/PivotMotionMagicAcceleration", 0);
    private DoublePreferenceConstant p_PivotMaxJerk = new DoublePreferenceConstant("Elevator/PivotMotionMagicJerk", 0);
    private DoublePreferenceConstant p_ElevatorMaxVelocity = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicVelocity", 0);
    private DoublePreferenceConstant p_ElevatorMaxAcceleration = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicAcceleration", 0);
    private DoublePreferenceConstant p_ElevatorMaxJerk = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicJerk", 0);

    private DoublePreferenceConstant p_pivotStowSpeed = new DoublePreferenceConstant("Elevator/PivotStowSpeed", 0.05);
    private DoublePreferenceConstant p_elevatorStowSpeed = new DoublePreferenceConstant("Elevator/ElevatorStowSpeed",
            0.05);

    private PIDPreferenceConstants p_PivotPIDPreferenceConstants = new PIDPreferenceConstants("Elevator/PivotPID");
    private PIDPreferenceConstants p_ElevatorPIDPreferenceConstants = new PIDPreferenceConstants(
            "Elevator/ElevatorPID");

    private final double kPivotMotorRotationToShooterAngle = 360.0 / 25.0;
    // TODO get this -> private final double kElevatorMotorToElevatorDistance =
    // 360.0 / 25.0;

    private final TalonFX m_pivotMotor = new TalonFX(Constants.ELEVATOR_ANGLER_MOTOR, Constants.CANIVORE_CANBUS);
    private final TalonFX m_elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR, Constants.RIO_CANBUS);
    private MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage m_elevatorRequest = new MotionMagicVoltage(0);
    private final Debouncer pivotDebouncer = new Debouncer(1, DebounceType.kRising);
    private final Debouncer elevatorDebouncer = new Debouncer(1, DebounceType.kRising);

    public Elevator() {

        configureTalons();
        p_PivotMaxVelocity.addChangeHandler((Double unused) -> configureTalons());
        p_PivotMaxAcceleration.addChangeHandler((Double unused) -> configureTalons());
        p_PivotMaxJerk.addChangeHandler((Double unused) -> configureTalons());
        p_PivotPIDPreferenceConstants.addChangeHandler((Double unused) -> configureTalons());
        p_ElevatorMaxVelocity.addChangeHandler((Double unused) -> configureTalons());
        p_ElevatorMaxAcceleration.addChangeHandler((Double unused) -> configureTalons());
        p_ElevatorMaxJerk.addChangeHandler((Double unused) -> configureTalons());
        p_ElevatorPIDPreferenceConstants.addChangeHandler((Double unused) -> configureTalons());

        m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureTalons() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.StatorCurrentLimit = 60;

        MotionMagicConfigs pivotMM = pivotConfig.MotionMagic;
        pivotMM.MotionMagicAcceleration = p_PivotMaxAcceleration.getValue();
        pivotMM.MotionMagicJerk = p_PivotMaxJerk.getValue();
        pivotMM.MotionMagicCruiseVelocity = p_PivotMaxVelocity.getValue();

        Slot0Configs pivotSlot0 = pivotConfig.Slot0;
        pivotSlot0.kP = p_PivotPIDPreferenceConstants.getKP().getValue();
        pivotSlot0.kI = p_PivotPIDPreferenceConstants.getKI().getValue();
        pivotSlot0.kD = p_PivotPIDPreferenceConstants.getKD().getValue();
        pivotSlot0.kS = 0.4;
        pivotSlot0.kV = p_PivotPIDPreferenceConstants.getKF().getValue();

        m_pivotMotor.getConfigurator().apply(pivotConfig);

        m_pivotMotor.setInverted(true);

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;

        MotionMagicConfigs elevatorMM = elevatorConfig.MotionMagic;
        elevatorMM.MotionMagicAcceleration = p_PivotMaxAcceleration.getValue();
        elevatorMM.MotionMagicJerk = p_PivotMaxJerk.getValue();
        elevatorMM.MotionMagicCruiseVelocity = p_PivotMaxVelocity.getValue();

        Slot0Configs elevatorSlot0 = elevatorConfig.Slot0;
        elevatorSlot0.kP = p_ElevatorPIDPreferenceConstants.getKP().getValue();
        elevatorSlot0.kI = p_ElevatorPIDPreferenceConstants.getKI().getValue();
        elevatorSlot0.kD = p_ElevatorPIDPreferenceConstants.getKD().getValue();
        elevatorSlot0.kS = 0.4;
        elevatorSlot0.kV = p_ElevatorPIDPreferenceConstants.getKF().getValue();

        m_elevatorMotor.getConfigurator().apply(elevatorConfig);
    }

    public void stow() {
        m_pivotMotor.setControl(new DutyCycleOut(-p_pivotStowSpeed.getValue()));
        if (pivotDebouncer.calculate(m_pivotMotor.getVelocity().getValueAsDouble() > -1)) {
            calibratePivot();
        }
        m_pivotMotor.setControl(new DutyCycleOut(-p_elevatorStowSpeed.getValue()));
        if (elevatorDebouncer.calculate(m_elevatorMotor.getVelocity().getValueAsDouble() > -1)) {
            calibrateElevator();
        }
    }

    public void setPivotPosition(double position) {
        m_pivotMotor.setControl(m_pivotRequest.withPosition(position / kPivotMotorRotationToShooterAngle));
    }

    public void setElevatorPosition(double position) {
        m_elevatorMotor.setControl(m_elevatorRequest.withPosition(position / kElevatorMotorRotationToElevatorDistance));
    }

    public void calibratePivot() {
        m_pivotMotor.setPosition(42 / kPivotMotorRotationToShooterAngle);
    }

    public void calibrateElevator() {
        m_elevatorMotor.setPosition(0);
    }

    public Command setPodiumFactory() {
        return new RunCommand(() -> setPivotPosition(p_pivotPodium.getValue()), this);
    }

    public Command setFlatFactory() {
        return new RunCommand(() -> setPivotPosition(p_pivotFlat.getValue()), this);
    }

    public Command calibratePivotFactory() {
        return new InstantCommand(() -> calibratePivot(), this).ignoringDisable(true);
    }

    public Command calibrateElevatorFactory() {
        return new InstantCommand(() -> calibrateElevator(), this).ignoringDisable(true);
    }

    public Command stowFactory() {
        return new RunCommand(this::stow, this).beforeStarting(() -> pivotDebouncer.calculate(false));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle",
                m_pivotMotor.getPosition().getValueAsDouble() * kPivotMotorRotationToShooterAngle);
    }
}
