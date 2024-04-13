// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class Autonomous {

    public static final String[] eiffelOptions = { "EiffelTower", "EiffelTowerBCD", "EiffelTowerBDC",
            "EiffelTowerCBD", "EiffelTowerCDB", "EiffelTowerDBC", "EiffelTowerDCB", "EiffelTowerEBC",
            "EiffelTowerECB" };

    public static SendableChooser<Command> eiffelChooser(CommandSwerveDrivetrain drivetrain) {
        SendableChooser<Command> chooser = new SendableChooser<>();

        chooser.setDefaultOption(eiffelOptions[0], drivetrain.getAutoPath(eiffelOptions[0]));

        for (int i = 1; i < eiffelOptions.length; i++) {
            chooser.addOption(eiffelOptions[i], drivetrain.getAutoPath(eiffelOptions[i]));
        }

        return chooser;
    }
}
