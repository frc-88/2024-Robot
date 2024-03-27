package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ros.bridge.CoprocessorBridge;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;

public class Lights extends SubsystemBase {
    private IntPreferenceConstant numLEDs = new IntPreferenceConstant("Number Of LEDs", 60);
    private int m_state = 0;
    private int counter = 0;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private boolean m_setAnim = false;

    private Animation m_toAnimate = null;
    private Animation m_lastAnimation = null;

    private CommandSwerveDrivetrain m_swerve;
    private Intake m_intake;
    private Elevator m_elevator;
    private Shooter m_shooter;
    private Climber m_climber;
    private CoprocessorBridge m_coprocessor;
    private Supplier<String> m_autoName;

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

    public Lights(CommandSwerveDrivetrain swerve, Intake intake, Elevator elevator,
            Shooter shooter, Climber climber, CoprocessorBridge coprocessor, Supplier<String> autoName) {
        m_swerve = swerve;
        m_intake = intake;
        m_elevator = elevator;
        m_shooter = shooter;
        m_climber = climber;
        m_coprocessor = coprocessor;
        m_autoName = autoName;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.On;
        m_candle.configAllSettings(configAll, 100);
    }

    public void noteSpinLeft() {
        m_toAnimate = new ColorFlowAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward);
    }

    public void noteSpinRight() {
        m_toAnimate = new ColorFlowAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), Direction.Backward);
    }

    public void holdingNote() {
        m_toAnimate = new LarsonAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), BounceMode.Center, 8);
    }

    public void intakingNote() {
        m_toAnimate = new StrobeAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue());
    }

    public void setFire() {
        m_toAnimate = new FireAnimation(1, 0.6, numLEDs.getValue(), 0.2, 0.2);
    }

    public void larsonColor(int r, int g, int b) {
        m_toAnimate = new LarsonAnimation(r, g, b, 0, 0.2, numLEDs.getValue(), BounceMode.Front, 5);
    }

    public void rainbow() {
        m_toAnimate = new RainbowAnimation(1, 0.7, numLEDs.getValue());
    }

    // TODO: test this animation to see if it truly works
    public void tiedye() {
        m_candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward, 0), 0);
        m_candle.animate(new ColorFlowAnimation(255, 165, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward, 10), 1);
        m_candle.animate(new ColorFlowAnimation(255, 255, 0, 0, 0.2, numLEDs.getValue(), Direction.Forward, 20), 2);
        m_candle.animate(new ColorFlowAnimation(0, 0, 255, 0, 0.2, numLEDs.getValue(), Direction.Forward, 30), 3);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            switch (m_state) {
                // TODO:put in subsystem stuff when integrated
                case 0: {
                    larsonColor(255, 0, 0);
                    // swerve goes here
                    if (true && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 1: {
                    larsonColor(255, 165, 0);
                    // elevator goes here
                    if (m_elevator.isElevatorReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 2: {
                    larsonColor(255, 255, 0);
                    // intake goes here
                    if (m_intake.isIntakeReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 3: {
                    larsonColor(0, 255, 0);
                    // indexer goes here
                    if (m_intake.isIndexerReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 4: {
                    larsonColor(0, 0, 255);
                    // shooter goes here
                    if (m_shooter.isShooterReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 5: {
                    larsonColor(0, 255, 255);
                    // climber goes here
                    if (m_climber.isClimberReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 6: {
                    larsonColor(143, 0, 255);
                    // ROS goes here
                    if (m_coprocessor.isCoprocessorReady().getAsBoolean() && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 7: {
                    larsonColor(255, 255, 255);
                    if (true && counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
                case 8: {
                    rainbow();
                    if (counter++ > 200) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
            }
        } else {
            if (m_intake.hasNoteInIndexer()) {
                holdingNote();
            } else if (m_intake.isIntakingNote()) {
                intakingNote();
            } else {
                tiedye();
            }
        }

        // if animation is equal to last one, don't clear
        if (m_toAnimate.equals(m_lastAnimation))

        {
            m_setAnim = false;
            // if animation if not equal to last one, clear animation
        } else if (!m_toAnimate.equals(m_lastAnimation) && m_lastAnimation != null) {
            m_lastAnimation = m_toAnimate;
            m_setAnim = true;
            // for the very first time when m_lastAnimation is null, don't clear.
        } else {
            m_lastAnimation = m_toAnimate;
            m_setAnim = false;
        }
        // if (m_setAnim) {
        // m_candle.clearAnimation(0);
        // m_setAnim = false;
        // }
        m_candle.animate(m_toAnimate);
    }

    public InstantCommand spinLeftFactory() {
        return new InstantCommand(() -> {
            noteSpinLeft();
        });
    }

    public InstantCommand spinRightFactory() {
        return new InstantCommand(() -> {
            noteSpinRight();
        });
    }

    public InstantCommand holdNoteFactory() {
        return new InstantCommand(() -> {
            holdingNote();
        });
    }

    public InstantCommand setFireFactory() {
        return new InstantCommand(() -> {
            setFire();
        });
    }

    public InstantCommand tieDyeFactory() {
        return new InstantCommand(() -> {
            tiedye();
        });
    }
}
