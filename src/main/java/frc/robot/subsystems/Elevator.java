package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    private DoublePreferenceConstant p_pivotAmp = new DoublePreferenceConstant("Elevator/PivotAmp", 0);
    private DoublePreferenceConstant p_elevatorAmp = new DoublePreferenceConstant("Elevator/ElevatorAmp", 0);
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
    private DoublePreferenceConstant p_elevatorGravityFeedForward = new DoublePreferenceConstant("Elevator/Gravity", 0);

    private DoublePreferenceConstant p_pivotStowSpeed = new DoublePreferenceConstant("Elevator/PivotStowSpeed", 0.05);
    private DoublePreferenceConstant p_elevatorStowSpeed = new DoublePreferenceConstant("Elevator/ElevatorStowSpeed",
            0.05);
    private DoublePreferenceConstant p_elevatorClimbPosition = new DoublePreferenceConstant(
            "Elevator/ElevatorClimbPosition", 0);
    private DoublePreferenceConstant p_elevatorPrepPosition = new DoublePreferenceConstant(
            "Elevator/ElevatorPrepPosition", 0);

    private PIDPreferenceConstants p_PivotPIDPreferenceConstants = new PIDPreferenceConstants("Elevator/PivotPID");
    private PIDPreferenceConstants p_ElevatorPIDPreferenceConstants = new PIDPreferenceConstants(
            "Elevator/ElevatorPID");

    private final double kPivotMotorRotationToShooterAngle = 360.0 / 25.0;
    private final double kElevatorMotorToElevatorDistance = (7.086614173228346 / 14);

    private final TalonFX m_pivotMotor = new TalonFX(Constants.ELEVATOR_ANGLER_MOTOR, Constants.CANIVORE_CANBUS);
    private final TalonFX m_elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR, Constants.RIO_CANBUS);
    private MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage m_elevatorRequest = new MotionMagicVoltage(0);
    private final Debouncer pivotDebouncer = new Debouncer(1, DebounceType.kRising);
    private final Debouncer elevatorDebouncer = new Debouncer(1, DebounceType.kRising);

    private double m_elevatorTarget;

    public Elevator() {

        configureTalons(0);
        p_PivotMaxVelocity.addChangeHandler(this::configureTalons);
        p_PivotMaxAcceleration.addChangeHandler(this::configureTalons);
        p_PivotMaxJerk.addChangeHandler(this::configureTalons);
        p_PivotPIDPreferenceConstants.addChangeHandler(this::configureTalons);
        p_ElevatorMaxVelocity.addChangeHandler(this::configureTalons);
        p_ElevatorMaxAcceleration.addChangeHandler(this::configureTalons);
        p_ElevatorMaxJerk.addChangeHandler(this::configureTalons);
        p_ElevatorPIDPreferenceConstants.addChangeHandler(this::configureTalons);
        p_elevatorGravityFeedForward.addChangeHandler(this::configureTalons);

        m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureTalons(double unused) {
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
        elevatorSlot0.kS = 0;
        elevatorSlot0.kV = p_ElevatorPIDPreferenceConstants.getKF().getValue();
        elevatorSlot0.kG = p_elevatorGravityFeedForward.getValue();
        elevatorSlot0.GravityType = GravityTypeValue.Elevator_Static;

        m_elevatorMotor.getConfigurator().apply(elevatorConfig);

        m_elevatorMotor.setInverted(true);
    }

    public boolean onTarget() {
        return Math.abs(m_elevatorMotor.getPosition().getValueAsDouble() * kElevatorMotorToElevatorDistance
                - m_elevatorTarget) < 2;
    }

    public void enableCoastMode() {
        m_elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void enableBrakeMode() {
        m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void pivotStow() {
        m_pivotMotor.setControl(new DutyCycleOut(-p_pivotStowSpeed.getValue()));
        if (pivotDebouncer.calculate(m_pivotMotor.getVelocity().getValueAsDouble() > -1)) {
            calibratePivot();
        }
    }

    public void elevatorStow() {
        m_elevatorMotor.setControl(new DutyCycleOut(-p_elevatorStowSpeed.getValue()));
        if (elevatorDebouncer.calculate(m_elevatorMotor.getVelocity().getValueAsDouble() > -1)) {
            calibrateElevator();
        }
    }

    public void setPivotPosition(double position) {
        m_pivotMotor.setControl(m_pivotRequest.withPosition(position / kPivotMotorRotationToShooterAngle));
    }

    public void setPivotPosition(DoubleSupplier position) {
        m_pivotMotor
                .setControl(m_pivotRequest.withPosition(position.getAsDouble() / kPivotMotorRotationToShooterAngle));
    }

    public void setElevatorPosition(double height) {
        m_elevatorTarget = height;
        m_elevatorMotor.setControl(m_elevatorRequest.withPosition(height / kElevatorMotorToElevatorDistance));
    }

    public void setElevatorPosition(DoubleSupplier height) {
        m_elevatorMotor
                .setControl(m_elevatorRequest.withPosition(height.getAsDouble() / kElevatorMotorToElevatorDistance));
    }

    public void calibratePivot() {
        m_pivotMotor.setPosition(42.0 / kPivotMotorRotationToShooterAngle);
    }

    public void calibrateElevator() {
        m_elevatorMotor.setPosition(27.7 / kElevatorMotorToElevatorDistance);
    }

    public Command elevatorPrepFactory() {
        return new RunCommand(() -> setElevatorPosition(p_elevatorPrepPosition.getValue()), this);
    }

    public Command climbFactory() {
        return new RunCommand(() -> {
            setElevatorPosition(p_elevatorClimbPosition.getValue());
            pivotStow();
        }, this);
    }

    public Command setAmpFactory() {
        return new RunCommand(() -> {
            setPivotPosition(() -> p_pivotAmp.getValue());
            setElevatorPosition(() -> p_elevatorAmp.getValue());
        }, this);
    }

    public Command setPodiumFactory() {
        return new RunCommand(() -> {
            setPivotPosition(() -> p_pivotPodium.getValue());
            elevatorStow();
        }, this);
    }

    public Command setFlatFactory() {
        return new RunCommand(() -> {
            setPivotPosition(() -> p_pivotFlat.getValue());
            elevatorStow();
        }, this);
    }

    public Command calibratePivotFactory() {
        return new InstantCommand(() -> calibratePivot(), this).ignoringDisable(true);
    }

    public Command calibrateElevatorFactory() {
        return new InstantCommand(() -> calibrateElevator(), this).ignoringDisable(true);
    }

    public Command stowFactory() {
        return new RunCommand(() -> {
            pivotStow();
            elevatorStow();
        }, this).beforeStarting(() -> {
            pivotDebouncer.calculate(false);
            elevatorDebouncer.calculate(false);
        });
    }

    public Command enableCoastModeFactory() {
        return new InstantCommand(() -> {
            enableCoastMode();
        }, this);
    }

    public Command enableBrakeModeFactory() {
        return new InstantCommand(() -> {
            enableBrakeMode();
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle",
                m_pivotMotor.getPosition().getValueAsDouble() * kPivotMotorRotationToShooterAngle);
        SmartDashboard.putNumber("Elevator Height",
                m_elevatorMotor.getPosition().getValueAsDouble() * kElevatorMotorToElevatorDistance);
    }
}