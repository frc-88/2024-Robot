// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;
import static frc.robot.Constants.LimelightConstants.*;

import java.sql.Driver;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.BotPoseProvider;
import frc.team88.ros.messages.geometry_msgs.Pose2D;

/*
 *     some think that I need
 * bright green glowing eyes to see
 *      but April guides me
 */

public class Limelight extends SubsystemBase implements BotPoseProvider {

    private String m_name;
    public double m_distance;
    private NetworkTable limelightTable;
    private Alliance alliance;
    public int fiducial;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    public Limelight(String name) {
        m_name = name;
        // limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
        // int[] speakertags = { 4, 7 };
        // LimelightHelpers.SetFiducialIDFiltersOverride(m_name, speakertags);
        limelightTable = NetworkTableInstance.getDefault().getTable(m_name);
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");

        // post to smart dashboard periodically
    }

    public double getTX() {
        return Math.toRadians(LimelightHelpers.getTX(m_name));
    }

    public double getTY() {
        return Math.toRadians(LimelightHelpers.getTY(m_name) + 25.5);
    }

    public double getFiducialID() {
        return fiducial;
    }

    public void limelightSwitch(int pipe) {
        NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
        limeLightPipe.setNumber(pipe);
    }

    public void setAprilTagPipeline() {
        limelightSwitch(0);
    }

    public boolean isAprilTagPipelineActive() {
        NetworkTableEntry limeLightPipe = limelightTable.getEntry("pipeline");
        return limeLightPipe.getInteger(0) == 0;
    }

    public Pose2d getBotPose() {
        if (LimelightHelpers.getFiducialID(m_name) > 0.0) {
            if (getAlliance() == DriverStation.Alliance.Red) {
                LimelightHelpers.PoseEstimate robotPosemt = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_name);
                return robotPosemt.pose;
            } else {
                LimelightHelpers.PoseEstimate robotPosemt = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(m_name);
                return robotPosemt.pose;
            }
        } else {
            return new Pose2d();
        }
    }

    public boolean isConnected() {
        return LimelightHelpers.getFiducialID(m_name) > 0.0;
    }

    @Override
    public void periodic() {
        Pose2d botPose = getBotPose();
        SmartDashboard.putNumber("LL:" + m_name + ":BotX", tx.getDouble(0.0));
        SmartDashboard.putNumber("LL:" + m_name + ":BotY", ty.getDouble(0.0));
        SmartDashboard.putNumber("LL:" + m_name + ":BotY1", getTY());
        SmartDashboard.putNumber("LL:" + m_name + ":BotYaw", botPose.getRotation().getDegrees());
        SmartDashboard.putNumber("LL:" + m_name + ":M_Distance", m_distance);
    }

    // Returns distance to the center of the speaker tag from the robot or -1 if not
    // found
    // public double getDistanceToCenterSpeakerTagFromRobot() {
    // // RawFiducial[] tags = fiducial;
    // LimelightHelpers.PoseEstimate speakerdis;
    // int[] redspeakerTagID = { 4 }; // Red Speaker Tag
    // int[] bluespeakerTagID = { 7 }; // Blue Speaker Tag
    // if (getAlliance() == DriverStation.Alliance.Red) {
    // LimelightHelpers.SetFiducialIDFiltersOverride(m_name, redspeakerTagID);
    // LimelightHelpers.PoseEstimate mt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
    // } else {
    // LimelightHelpers.SetFiducialIDFiltersOverride(m_name, bluespeakerTagID);
    // LimelightHelpers.PoseEstimate mt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // }

    // }

    // return -1;
    // }

    private Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        return alliance;
    }
}
