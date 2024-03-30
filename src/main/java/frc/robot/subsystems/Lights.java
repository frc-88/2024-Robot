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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ros.bridge.CoprocessorBridge;
import frc.robot.util.Aiming;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;

public class Lights extends SubsystemBase {
    private IntPreferenceConstant numLEDs = new IntPreferenceConstant("Number Of LEDs", 60);
    private int m_state = 0;
    private int counter = 0;
    private int increments = 0;
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
    private boolean m_clearAnim = true;
    private boolean m_setAnim = true;
    private boolean m_shooting = false;
    private boolean m_tiedye = false;

    private Animation m_toAnimate = null;
    private Animation m_lastAnimation = null;

    private CommandSwerveDrivetrain m_swerve;
    private Intake m_intake;
    private Elevator m_elevator;
    private Shooter m_shooter;
    private Climber m_climber;
    private CoprocessorBridge m_coprocessor;
    private Aiming m_aiming;
    private Supplier<String> m_autoName;

    private boolean m_colorSet = false;

    class Colors {
        int r, g, b;

        private Colors(int red, int green, int blue) {
            this.r = red;
            this.g = green;
            this.b = blue;
        }
    }

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
            Shooter shooter, Climber climber, CoprocessorBridge coprocessor, Aiming aiming, Supplier<String> autoName) {
        m_swerve = swerve;
        m_intake = intake;
        m_elevator = elevator;
        m_shooter = shooter;
        m_climber = climber;
        m_coprocessor = coprocessor;
        m_aiming = aiming;
        m_autoName = autoName;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.On;
        m_candle.configAllSettings(configAll, 100);
        m_candle.configLEDType(LEDStripType.RGB, 300);
    }

    private Animation noteSpinLeft = new ColorFlowAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue(), Direction.Forward);
    private Animation noteSpinRight = new ColorFlowAnimation(165, 0, 0, 255, 0.2, numLEDs.getValue(),
            Direction.Backward);
    private Animation holdingNote = new ColorFlowAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue(), Direction.Forward);
    private Animation intakingNote = new StrobeAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue());
    private Animation setFire = new FireAnimation(1, 0.9, numLEDs.getValue(), 0.4, 0.4);
    private Animation rainBow = new RainbowAnimation(1, 0.7, numLEDs.getValue());

    public void noteSpinLeft() {
        m_setAnim = true;
        m_toAnimate = noteSpinLeft;
    }

    public void noteSpinRight() {
        m_setAnim = true;
        m_toAnimate = noteSpinRight;
    }

    public void holdingNote() {
        m_setAnim = true;
        m_toAnimate = holdingNote;
    }

    public void intakingNote() {
        m_setAnim = true;
        m_toAnimate = intakingNote;
    }

    public void setFire() {
        m_setAnim = true;
        m_toAnimate = setFire;
    }

    public void larsonColor(int r, int g, int b) {
        m_toAnimate = new LarsonAnimation(r, g, b, 0, 0.2, numLEDs.getValue(), BounceMode.Front, 8);
    }

    public void setLED(int r, int g, int b) {
        m_setAnim = false;
        m_clearAnim = true;
        m_candle.clearAnimation(0);
        m_candle.setLEDs(r, g, b);
    }

    public void disableLED() {
        m_setAnim = true;
    }

    public void rainbow() {
        m_setAnim = true;
        m_toAnimate = rainBow;
    }

    // TODO: test this animation to see if it truly works
    public void tiedye(boolean status) {
        m_tiedye = status;
    }

    @Override
    public void periodic() {
        if (m_setAnim == true) {
            m_tiedye = false;
        }
        if (m_tiedye == true) {
            m_candle.setLEDs(255, 0, 0, 0, ((0 + increments) % numLEDs.getValue()), numLEDs.getValue() / 5);
            m_candle.setLEDs(255, 165, 0, 0, (((0 + numLEDs.getValue() / 5) + increments) % numLEDs.getValue()),
                    numLEDs.getValue() / 5);
            m_candle.setLEDs(255, 255, 0, 0, (((0 + numLEDs.getValue() * 2 / 5) + increments) % numLEDs.getValue()),
                    numLEDs.getValue() / 5);
            m_candle.setLEDs(0, 0, 255, 0, (((0 + numLEDs.getValue() * 3 / 5) + increments) % numLEDs.getValue()),
                    numLEDs.getValue() / 5);
            m_candle.setLEDs(255, 255, 255, 0, (((0 + numLEDs.getValue() * 4 / 5) + increments) % numLEDs.getValue()),
                    numLEDs.getValue() / 5);
            increments++;
        }

        if (DriverStation.isDisabled())

        {
            switch (m_state) {
                case 0: {
                    if (!m_colorSet) {
                        // blue
                        larsonColor(0, 0, 255);
                        m_colorSet = true;
                    }
                    if (m_swerve.isSwerveReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 1: {
                    if (!m_colorSet) {
                        // pink
                        larsonColor(165, 0, 255);
                        m_colorSet = true;
                    }
                    if (m_elevator.isElevatorReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 2: {
                    if (!m_colorSet) {
                        // orange
                        larsonColor(255, 50, 0);
                        m_colorSet = true;
                    }
                    if (m_intake.isIntakeReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 3: {
                    if (!m_colorSet) {
                        // red
                        larsonColor(255, 0, 0);
                        m_colorSet = true;
                    }
                    if (m_intake.isIndexerReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 4: {
                    if (!m_colorSet) {
                        // green
                        larsonColor(0, 255, 0);
                        m_colorSet = true;
                    }
                    if (m_shooter.isShooterReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 5: {
                    if (!m_colorSet) {
                        // yellow
                        larsonColor(255, 255, 0);
                        m_colorSet = true;
                    }
                    if (m_climber.isClimberReady() && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 6: {
                    if (!m_colorSet) {
                        // light blue
                        larsonColor(0, 255, 143);
                        m_colorSet = true;
                    }
                    if (m_coprocessor.isCoprocessorReady(m_aiming.getROSPose()) && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 7: {
                    if (!m_colorSet) {
                        // white
                        larsonColor(255, 255, 255);
                        m_colorSet = true;
                    }
                    if (!m_autoName.get().equals("Wait") && counter++ > 50) {
                        m_state++;
                        counter = 0;
                        m_colorSet = false;
                    }
                    break;
                }
                case 8: {
                    rainbow();
                    if (counter++ > 50) {
                        m_state++;
                        counter = 0;
                    }
                    break;
                }
            }
        } else {
            if (m_intake.hasNoteInIndexer() && !m_shooting) {
                setLED(255, 0, 0);
            } else if (m_intake.isIntakingNote()) {
                intakingNote();
            } else if (m_shooting) {
                setFire();
            } else {
                rainbow();
            }
        }

        // if animation is equal to last one, don't clear
        if (m_toAnimate.equals(m_lastAnimation))

        {
            m_clearAnim = false;
            // if animation if not equal to last one, clear animation
        } else if (!m_toAnimate.equals(m_lastAnimation) && m_lastAnimation != null) {
            m_lastAnimation = m_toAnimate;
            m_clearAnim = true;
            // for the very first time when m_lastAnimation is null, don't clear.
        } else {
            m_lastAnimation = m_toAnimate;
            m_clearAnim = false;
        }
        if (m_clearAnim) {
            m_candle.clearAnimation(0);
            m_clearAnim = false;
        }
        if (m_setAnim) {
            m_candle.animate(m_toAnimate);
        }
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
            tiedye(true);
        });
    }

    public InstantCommand setShootingFactory(boolean isShooting) {
        return new InstantCommand(() -> {
            m_shooting = isShooting;
        });
    }

    public InstantCommand setLEDFactory(int r, int g, int b) {
        return new InstantCommand(() -> setLED(r, g, b), this);
    }
}
