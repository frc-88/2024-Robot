package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon {
    private DoublePreferenceConstant p_PigeonYawCalibration;
    private DoublePreferenceConstant p_PigeonRollCalibration; 
    private DoublePreferenceConstant p_PigeonPitchCalibration;
    private DoublePreferenceConstant p_GyroXSensitivityScalar;
    private DoublePreferenceConstant p_GyroYSensitivityScalar;
    private DoublePreferenceConstant p_GyroZSensitivityScalar;
    private final double GRAVITY = 9.81;
    private Pigeon2 base;
    
    public Pigeon(int deviceID) {
        p_PigeonYawCalibration = new DoublePreferenceConstant("Pigeon" + deviceID + "/InitialYaw", 0);
        p_PigeonRollCalibration = new DoublePreferenceConstant("Pigeon" + deviceID + "/InitialRoll", 0);
        p_PigeonPitchCalibration = new DoublePreferenceConstant("Pigeon" + deviceID + "/InitialPitch", 0);
        p_GyroXSensitivityScalar = new DoublePreferenceConstant("Pigeon" + deviceID + "/GyroXSensitivity", 0);
        p_GyroYSensitivityScalar = new DoublePreferenceConstant("Pigeon" + deviceID + "/GyroYSensitivity", 0);
        p_GyroZSensitivityScalar = new DoublePreferenceConstant("Pigeon" + deviceID + "/GyroZSensitivity", 0);
        base = new Pigeon2(deviceID);
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
        base.setYaw(0);
    }
    
    public double getAngle() {
        return base.getAngle();
    }
    
    public double getYaw() {
        return base.getYaw().getValue();
    }
    
    public double getRoll() {
        return base.getRoll().getValue();
    }
    
    public double getPitch() {
        return base.getPitch().getValue();
    }
    
    public Rotation2d getRotation2d() {
        return base.getRotation2d();
    }

    public double getAccelerationX() {
        return base.getAccelerationX().getValue() * GRAVITY;
    }
    
    public double getAccelerationY() {
        return base.getAccelerationY().getValue() * GRAVITY;
    }
    
    public double getAccelerationZ() {
        return base.getAccelerationZ().getValue() * GRAVITY;
    }
   
    public double getYawRate() {
        return base.getRate();
    }

    public void reset() {
        base.reset();
    }

}
    