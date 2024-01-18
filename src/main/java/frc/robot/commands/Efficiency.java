
package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Efficiency {
    static PowerDistribution power_d = new PowerDistribution(0,ModuleType.kCTRE);

    public static void battery(){
      double energy = power_d.getTotalEnergy();
      SmartDashboard.putNumber( "total_energy", energy);
      

    }
}
