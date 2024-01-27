package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Indexer extends SubsystemBase {
    final TalonFX m_Indexer = new TalonFX(9);

    private double talonFree = 6380;




    private DoublePreferenceConstant indexerSpeed =
  new DoublePreferenceConstant("shooter/indexer/speed", 0);

  public Indexer(){
    m_Indexer.setInverted(true);
  }

  public void startIndexer() {
    m_Indexer.set(indexerSpeed.getValue()/talonFree);
    }
    public void stopIndexer() {
        m_Indexer.set(0);
        } 
        public Command runIndexerCommand(){
    return new RunCommand(() -> {startIndexer();}, this);
  }
    public Command stopIndexerCommand(){
        return new RunCommand(() -> {stopIndexer();}, this);
    }
        @Override
        public void periodic() {
            // This method will be called once per scheduler run
        }
}
