package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Pigeon {
    private DoublePreferenceConstant p_PigeonYawCalibration = new DoublePreferenceConstant("Pigeon/InitialYaw", 0);
    private DoublePreferenceConstant p_PigeonRollCalibration = new DoublePreferenceConstant("Pigeon/InitialRoll", 0);
    private DoublePreferenceConstant p_PigeonPitchCalibration = new DoublePreferenceConstant("Pigeon/InitialPitch", 0);
    private DoublePreferenceConstant p_GyroXSensitivityScalar = new DoublePreferenceConstant("Pigeon/GyroSensitivityX", 0);
    private DoublePreferenceConstant p_GyroYSensitivityScalar = new DoublePreferenceConstant("Pigeon/GyroSensitivityY", 0);
    private DoublePreferenceConstant p_GyroZSensitivityScalar = new DoublePreferenceConstant("Pigeon/GyroSensitivityZ", 0);
    private final double GRAVITY = 9.81;
    private Pigeon2 base;
    
    public Pigeon(int deviceID) {
        this.base = new Pigeon2(deviceID);
        configurePigeon();
    }

    public void configurePigeon() {
        Pigeon2Configuration pigeonConfiguration = new Pigeon2Configuration();
        pigeonConfiguration.MountPose.MountPoseYaw = p_PigeonYawCalibration.getValue();
        pigeonConfiguration.MountPose.MountPoseRoll = p_PigeonRollCalibration.getValue();
        pigeonConfiguration.MountPose.MountPosePitch = p_PigeonPitchCalibration.getValue();
        pigeonConfiguration.GyroTrim.GyroScalarX = p_GyroXSensitivityScalar.getValue();
        pigeonConfiguration.GyroTrim.GyroScalarY = p_GyroYSensitivityScalar.getValue();
        pigeonConfiguration.GyroTrim.GyroScalarZ = p_GyroZSensitivityScalar.getValue();

        this.base.getConfigurator().apply(pigeonConfiguration);
    }

    public void calibratePigeon() {
        this.base.setYaw(0);
    }
    
    public double getAngle() {
        return this.base.getAngle();
    }
    
    public double getYaw() {
        return this.base.getYaw().getValue();
    }
    
    public double getRoll() {
        return this.base.getRoll().getValue();
    }
    
    public double getPitch() {
        return this.base.getPitch().getValue();
    }
    
    public Rotation2d getRotation2d() {
        return this.base.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return this.base.getRotation3d();
    }

    public double getAccelerationX() {
        return this.base.getAccelerationX().getValue() * GRAVITY;
    }
    
    public double getAccelerationY() {
        return this.base.getAccelerationY().getValue() * GRAVITY;
    }
    
    public double getAccelerationZ() {
        return this.base.getAccelerationZ().getValue() * GRAVITY;
    }
   
    public double getYawRate() {
        return this.base.getRate();
    }

    public void reset() {
        this.base.reset();
    }

}
    