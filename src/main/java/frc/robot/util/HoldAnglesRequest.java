package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class HoldAnglesRequest implements SwerveRequest {

    /**
     * The direction to point the modules toward.
     * This direction is still optimized to what the module was previously at.
     */
    public Rotation2d ModuleDirections[] = new Rotation2d[] {
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d(),
            new Rotation2d()
    };
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

        for (int i = 0; i < modulesToApply.length; ++i) {
            SwerveModuleState state = new SwerveModuleState(0, ModuleDirections[i]);
            modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    public void setModuleDirections(Rotation2d... directions) {
        this.ModuleDirections = directions;
    }
}
