package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;

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
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(config), "Failed to configure CANCoder");
            CtreUtils.checkCtreError(
                    StatusSignal.setUpdateFrequencyForAll(periodMilliseconds,
                        (BaseStatusSignal)encoder.getPosition(),
                        (BaseStatusSignal)encoder.getVelocity()
                        ),
                    "Failed to configure Falcon status frame period"
            );



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
            double angle = Math.toRadians(encoder.getAbsolutePosition().getValueAsDouble());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public boolean isPresent() {
            return !(encoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Red
                || encoder.getMagnetHealth().getValue() == MagnetHealthValue.Magnet_Invalid
                || encoder.clearStickyFault_Hardware() == StatusCode.SensorNotPresent);
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}