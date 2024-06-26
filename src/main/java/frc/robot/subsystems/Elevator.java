package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
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
import frc.robot.util.Aiming;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

// stay low when we go
// bringing notes to new levels
// go up to get down

public class Elevator extends SubsystemBase {
    private DoublePreferenceConstant p_pivotPrime = new DoublePreferenceConstant("Elevator/Prime", 90);
    private DoublePreferenceConstant p_pivotFlat = new DoublePreferenceConstant("Elevator/Flat", 42.0);
    private DoublePreferenceConstant p_pivotAmp = new DoublePreferenceConstant("Elevator/PivotAmp", 110);
    private DoublePreferenceConstant p_elevatorAmp = new DoublePreferenceConstant("Elevator/ElevatorAmp", 43);
    private DoublePreferenceConstant p_PivotMaxVelocity = new DoublePreferenceConstant(
            "Elevator/PivotMotionMagicVelocity", 100);
    private DoublePreferenceConstant p_PivotMaxAcceleration = new DoublePreferenceConstant(
            "Elevator/PivotMotionMagicAcceleration", 200);
    private DoublePreferenceConstant p_PivotMaxJerk = new DoublePreferenceConstant("Elevator/PivotMotionMagicJerk",
            10000);
    private DoublePreferenceConstant p_ElevatorMaxVelocity = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicVelocity", 100);
    private DoublePreferenceConstant p_ElevatorMaxAcceleration = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicAcceleration", 200);
    private DoublePreferenceConstant p_ElevatorMaxJerk = new DoublePreferenceConstant(
            "Elevator/ElevatorMotionMagicJerk", 10000);
    private DoublePreferenceConstant p_elevatorGravityFeedForward = new DoublePreferenceConstant("Elevator/Gravity",
            0.75);

    private DoublePreferenceConstant p_pivotStowSpeed = new DoublePreferenceConstant("Elevator/PivotStowSpeed", 0.05);
    private DoublePreferenceConstant p_elevatorStowSpeed = new DoublePreferenceConstant("Elevator/ElevatorStowSpeed",
            0.075);
    private DoublePreferenceConstant p_elevatorClimbPosition = new DoublePreferenceConstant(
            "Elevator/ElevatorClimbPosition", 47);
    private DoublePreferenceConstant p_elevatorPrepPosition = new DoublePreferenceConstant(
            "Elevator/ElevatorPrepPosition", 32);
    private DoublePreferenceConstant p_elevatorSourceHeight = new DoublePreferenceConstant(
            "Elevator/ElevatorSourceHeight", 32.0);

    private PIDPreferenceConstants p_PivotPIDPreferenceConstants = new PIDPreferenceConstants("Elevator/PivotPID", 50.0,
            0.0, 0.0,
            0.1, 0.0, 0.0, 0.0, 0.0);
    private PIDPreferenceConstants p_ElevatorPIDPreferenceConstants = new PIDPreferenceConstants(
            "Elevator/ElevatorPID", 10.0, 0.0, 0.2,
            0.14, 0.0, 0.0, 0.0, 0.0);

    private final double kPivotMotorRotationToShooterAngle = 360.0 / 25.0;
    private final double kElevatorMotorToElevatorDistance = (7.086614173228346 / 14);

    private final TalonFX m_pivotMotor = new TalonFX(Constants.ELEVATOR_ANGLER_MOTOR, Constants.RIO_CANBUS);
    private final TalonFX m_elevatorMotor = new TalonFX(Constants.ELEVATOR_MOTOR, Constants.RIO_CANBUS);
    private MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0);
    private MotionMagicVoltage m_elevatorRequest = new MotionMagicVoltage(0);
    private final Debouncer pivotDebouncer = new Debouncer(1, DebounceType.kRising);
    private final Debouncer elevatorDebouncer = new Debouncer(1, DebounceType.kRising);

    private double m_elevatorTarget;
    private boolean m_pivotCalibrated = false;
    private boolean m_elevatorCalibrated = false;

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

    public boolean isPivotCalibrated() {
        return m_pivotCalibrated;
    }

    public boolean isElevatorCalibrated() {
        return m_elevatorCalibrated;
    }

    public boolean isElevatorNotDown() {
        return m_elevatorMotor.getPosition().getValueAsDouble()
                * kElevatorMotorToElevatorDistance > (Constants.ELEVATOR_BOTTOM + 1.0);
    }

    public boolean elevatorOnTarget() {
        return Math.abs(m_elevatorMotor.getPosition().getValueAsDouble() * kElevatorMotorToElevatorDistance
                - m_elevatorTarget) < 2;
    }

    public boolean pivotOnTarget(double position, double tolerance) {
        return Math.abs(m_pivotMotor.getPosition().getValueAsDouble() * kPivotMotorRotationToShooterAngle
                - position) < tolerance;
    }

    public boolean pivotOnTarget(DoubleSupplier position, double tolerance) {
        return pivotOnTarget(position.getAsDouble(), tolerance);
    }

    public boolean pivotOnTargetForAmp() {
        return pivotOnTarget(p_pivotAmp.getValue(), 2.0);
    }

    public boolean isElevatorUp() {
        return Math.abs(m_elevatorMotor.getPosition().getValueAsDouble() * kElevatorMotorToElevatorDistance
                - p_elevatorClimbPosition.getValue()) < 1.0;
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
        if (m_pivotCalibrated) {
            m_pivotMotor.setControl(
                    m_pivotRequest.withPosition((Constants.PIVOT_BOTTOM + 0.7) / kPivotMotorRotationToShooterAngle));
        } else {
            m_pivotMotor.setControl(new DutyCycleOut(-p_pivotStowSpeed.getValue()));
            if (pivotDebouncer.calculate(m_pivotMotor.getVelocity().getValueAsDouble() > -1)) {
                calibratePivot();
                m_pivotCalibrated = true;
            }
        }
    }

    public void elevatorStow() {
        if (m_elevatorCalibrated) {
            m_elevatorMotor.setControl(m_elevatorRequest
                    .withPosition((Constants.ELEVATOR_BOTTOM + 0.3) / kElevatorMotorToElevatorDistance));
        } else {
            m_elevatorMotor.setControl(new DutyCycleOut(-p_elevatorStowSpeed.getValue()));
            if (elevatorDebouncer.calculate(m_elevatorMotor.getVelocity().getValueAsDouble() > -1)) {
                calibrateElevator();
                m_elevatorCalibrated = true;
            }
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
        m_elevatorTarget = height.getAsDouble();
        m_elevatorMotor
                .setControl(m_elevatorRequest.withPosition(height.getAsDouble() / kElevatorMotorToElevatorDistance));
    }

    public void calibratePivot() {
        m_pivotMotor.setPosition(42.0 / kPivotMotorRotationToShooterAngle);
    }

    public void calibrateElevator() {
        m_elevatorMotor.setPosition(Constants.ELEVATOR_BOTTOM / kElevatorMotorToElevatorDistance);
    }

    public void holdPosition() {
        m_elevatorMotor.setControl(new DutyCycleOut(0.0));
        m_pivotMotor.setControl(new DutyCycleOut(0.0));
    }

    public boolean areElevatorAndPivotDown() {
        return m_elevatorMotor.getPosition().getValueAsDouble() * kElevatorMotorToElevatorDistance
                - Constants.ELEVATOR_BOTTOM < 1.0
                && m_pivotMotor.getPosition().getValueAsDouble() * kPivotMotorRotationToShooterAngle
                        - Constants.PIVOT_BOTTOM < 1.0;
    }

    public boolean isElevatorReady() {
        return m_elevatorMotor.isAlive()
                && m_pivotMotor.isAlive();
    }

    public Command elevatorDownFactory() {
        return new RunCommand(() -> {
            setElevatorPosition(Constants.ELEVATOR_BOTTOM + 0.8);
            setPivotPosition(42.0);
        }, this);
    }

    public Command elevatorPrepFactory() {
        return new RunCommand(() -> setElevatorPosition(p_elevatorPrepPosition.getValue()), this);
    }

    public Command goToAnlgeFactory(double position) {
        return new RunCommand(() -> setPivotPosition(position), this);
    }

    public Command climbFactory() {
        return new RunCommand(() -> {
            setElevatorPosition(p_elevatorClimbPosition.getValue());
            pivotStow();
        }, this);
    }

    public Command trapFactory() {
        return new RunCommand(() -> {
            setElevatorPosition(p_elevatorClimbPosition.getValue());
            setPivotPosition(() -> p_pivotAmp.getValue());
        }, this);
    }

    public Command setAmpFactory() {
        return new RunCommand(() -> {
            setPivotPosition(() -> p_pivotAmp.getValue());
            setElevatorPosition(() -> p_elevatorAmp.getValue());
        }, this);
    }

    public Command primeFactory() {
        return new RunCommand(() -> {
            setPivotPosition(() -> p_pivotPrime.getValue());
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

    public Command goToAimingPosition(double position) {
        return new RunCommand(() -> setPivotPosition(position), this);
    }

    public Command goToAimingPosition(DoubleSupplier position) {
        return new RunCommand(() -> setPivotPosition(position), this);
    }

    public Command holdPositionFactory() {
        return new RunCommand(() -> holdPosition(), this);
    }

    public Command sourceIntakeFactory() {
        return new RunCommand(() -> {
            pivotStow();
            setElevatorPosition(p_elevatorSourceHeight.getValue());
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
