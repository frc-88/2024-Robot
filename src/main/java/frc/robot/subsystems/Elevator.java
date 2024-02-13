package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Elevator extends SubsystemBase {
    private DoublePreferenceConstant p_pivotStow = new DoublePreferenceConstant("Elevator/Stow", 0);
    private DoublePreferenceConstant p_pivotFlat = new DoublePreferenceConstant("Elevator/Flat", 50);
    private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Elevator/MotionMagicVelocity", 0);
    private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant(
            "Elevator/MotionMagicAcceleration", 0);
    private DoublePreferenceConstant p_maxJerk = new DoublePreferenceConstant("Elevator/MotionMagicJerk", 0);
    private PIDPreferenceConstants p_PIDPreferenceConstants = new PIDPreferenceConstants("Elevator/PID");
    private final double kMotorRotationToShooterAngle = 360.0 / 25.0;

    private final TalonFX m_pivotMotor = new TalonFX(Constants.ELEVATOR_ANGLER_MOTOR, Constants.CANIVORE_CANBUS);
    private MotionMagicVoltage m_pivotRequest = new MotionMagicVoltage(0);

    public Elevator() {
        configureTalons();
        p_maxVelocity.addChangeHandler((Double unused) -> configureTalons());
        p_maxAcceleration.addChangeHandler((Double unused) -> configureTalons());
        p_maxJerk.addChangeHandler((Double unused) -> configureTalons());
        p_PIDPreferenceConstants.addChangeHandler((Double unused) -> configureTalons());
    }

    private void configureTalons() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicAcceleration = p_maxAcceleration.getValue();
        mm.MotionMagicJerk = p_maxJerk.getValue();
        mm.MotionMagicCruiseVelocity = p_maxVelocity.getValue();

        Slot0Configs slot0 = config.Slot0;
        slot0.kP = p_PIDPreferenceConstants.getKP().getValue();
        slot0.kI = p_PIDPreferenceConstants.getKI().getValue();
        slot0.kD = p_PIDPreferenceConstants.getKD().getValue();
        slot0.kS = .4;
        slot0.kV = p_PIDPreferenceConstants.getKF().getValue();

        m_pivotMotor.getConfigurator().apply(config);

        m_pivotMotor.setInverted(true);
    }

    public void setPosition(double position) {
        m_pivotMotor.setControl(m_pivotRequest.withPosition(position / kMotorRotationToShooterAngle));
    }

    public void calibrateAngle() {
        m_pivotMotor.setPosition(0);
    }

    public Command setStowFactory() {
        return new RunCommand(() -> setPosition(p_pivotStow.getValue()), this);
    }

    public Command setFlatFactory() {
        return new RunCommand(() -> setPosition(p_pivotFlat.getValue()), this);
    }

    public Command calibrateAngleFactory() {
        return new InstantCommand(() -> calibrateAngle(), this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle",
                m_pivotMotor.getPosition().getValueAsDouble() * kMotorRotationToShooterAngle);
    }
}
