package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;

public class Lights extends SubsystemBase{
    private IntPreferenceConstant numLEDs = new IntPreferenceConstant("Number Of LEDs", 60);
    private int m_state = 0;
    private int counter = 0;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private boolean m_setAnim = false;
    
    private Animation m_toAnimate = null;
    private Animation m_lastAnimation = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }

    // public Lights(SwerveDrive swerve, Intake intake, Elevator elevator, Supplier<String> autoName) {

    // }

    public void noteSpinLeft() {
        m_toAnimate = new ColorFlowAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward);
        m_setAnim = true; 
    }

    public void noteSpinRight() {
        m_toAnimate = new ColorFlowAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), Direction.Backward);
        m_setAnim = true;
    }

    // private SwerveDrive m_werve
    // private Intake m_intake
    // private Elevator m_elevator


    @Override
    public void periodic() {
        if(DriverStation.isDisabled()) {
            switch (m_state) {
                //TODO:put in swerve stuff when integrated
            }
        }
        if (m_setAnim) {
            m_candle.clearAnimation(0);
            m_setAnim = false;
        }
        m_candle.animate(m_toAnimate, 0);
    }
}
