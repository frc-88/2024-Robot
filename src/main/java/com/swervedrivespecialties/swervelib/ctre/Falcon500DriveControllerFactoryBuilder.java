package com.swervedrivespecialties.swervelib.ctre;


import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;
    private static final double WHEEL_RADIUS = 0.0508;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer deviceID, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
                motorConfiguration.Voltage.PeakForwardVoltage = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            TalonFX motor = new TalonFX(deviceID);
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500");

            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }

            motor.setNeutralMode(NeutralModeValue.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted() ? true : false);
            //no method for this one in pheonix6
            motor.setSensorPhase(true);

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS
                    ),
                    "Failed to configure Falcon status frame period"
            );

            return new ControllerImplementation(motor, sensorVelocityCoefficient, sensorPositionCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double sensorPositionCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, double sensorPositionCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.sensorPositionCoefficient = sensorPositionCoefficient;
        }

        @Override
        public TalonFX getMotor() {
            return motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.set(voltage / nominalVoltage);
        }

        @Override
        public double getStateVelocity() {
            return motor.getVelocity().getValueAsDouble();
        }

        @Override
        public double getDistance() {
            return motor.getPosition().getValueAsDouble();
        }
    }
}