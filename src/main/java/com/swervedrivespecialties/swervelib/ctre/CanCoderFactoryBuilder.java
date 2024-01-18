package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class CanCoderFactoryBuilder {
    private SensorDirectionValue direction = SensorDirectionValue.Clockwise_Positive;
    private AbsoluteSensorRangeValue sensorRangeValue = AbsoluteSensorRangeValue.Unsigned_0To1;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(SensorDirectionValue direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = sensorRangeValue;
            config.MagnetSensor.MagnetOffset = Math.toDegrees(configuration.getOffset());
            config.MagnetSensor.SensorDirection = direction;

            CANcoder encoder = new CANcoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public boolean isPresent() {
            return !(encoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED
                || encoder.getMagnetFieldStrength() == MagnetFieldStrength.Invalid_Unknown
                || encoder.getLastError() == ErrorCode.SensorNotPresent);
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}