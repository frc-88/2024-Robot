// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.ros.bridge.BridgePublisher;
import frc.team88.ros.bridge.ROSNetworkTablesBridge;
import frc.team88.ros.conversions.TFListenerCompact;
import frc.team88.ros.messages.visualization_msgs.MarkerArray;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.ros.bridge.BagManager;
import frc.robot.ros.bridge.CoprocessorBridge;
import frc.robot.ros.bridge.TagSubscriber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Aiming;
import frc.robot.util.DriveUtils;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class RobotContainer {
    private final Aiming m_aiming = new Aiming();
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandGenericHID buttonBox = new CommandGenericHID(1); // The buttons???
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(m_aiming); // My drivetrain
    private String m_autoCommandName = "Wait";
    private final Shooter m_shooter = new Shooter();
    private final Elevator m_elevator = new Elevator();
    private final Intake m_intake = new Intake(m_elevator::areElevatorAndPivotDown);
    private Climber m_climber = new Climber();
    private Lights m_lights;

    private Command m_autoCommand = new SequentialCommandGroup(
            new WaitCommand(4),
            m_intake.shootIndexerFactory().withTimeout(2),
            new RunCommand(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds(1, 0, 0)), drivetrain).withTimeout(2),
            drivetrain.applyRequest(drivetrain.SnapToAngleRequest(joystick)))
            .alongWith(m_shooter.runShooterFactory().withTimeout(6).andThen(m_shooter.stopShooterFactory()));

    private Command climb(boolean trap) {
        return new SequentialCommandGroup(m_elevator.climbFactory().alongWith(m_climber.keepArmsPreppedFactory())
                .until(m_elevator::isElevatorUp),
                m_climber.climbFactory().alongWith(trap ? m_elevator.trapFactory() : m_elevator.climbFactory()))
                .unless(drivetrain.tipping());
    }

    private Command intakeFromSource() {
        return new SequentialCommandGroup(new InstantCommand(m_intake::disableAutoMode), new WaitCommand(0.1),
                m_shooter.runSourceIntakeFactory().alongWith(m_elevator.sourceIntakeFactory(),
                        m_intake.sourceIntakeFactory()).until(m_intake.hasNoteDebounced()),
                m_shooter.runSourceIntakeFactory().alongWith(m_elevator.sourceIntakeFactory(),
                        m_intake.sourceIntakeFactory()).until(m_intake.hasNoteDebounced().negate()),
                m_intake.intakeFactory().deadlineWith(m_shooter.stopShooterFactory(), m_elevator.stowFactory()));
    }

    private Command goblinModeFactory() {
        return new ParallelCommandGroup(m_intake.goblinModeFactory(), m_shooter.runShuttlePassFactory(() -> false),
                drivetrain.aimAtAmpDumpingGroundFactory(() -> false));
    }

    private DoubleSupplier joystickAimAtHeading() {
        return () -> joystick.getLeftX() < 0.1 && joystick.getLeftY() < 0.1
                ? drivetrain.getSupplierCurrentRobotAngle().getAsDouble()
                : Units.radiansToDegrees(Math.atan2(joystick.getLeftX(), joystick.getLeftY())) + 180;
    }

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps, drivetrain);
    private TFListenerCompact tfListenerCompact;
    private BagManager bagManager;
    @SuppressWarnings("unused")
    private CoprocessorBridge coprocessorBridge;
    private double indexerStart = m_intake.getIndexerPosition();
    private boolean readyToCoast = false;
    private boolean coasting = false;

    public RobotContainer() {
        DataLogManager.start();
        configureRosNetworkTablesBridge();
        configureDriverController();
        configureButtonBox();
        configureBindings();

        // PathPlanner Named Commands
        NamedCommands.registerCommand("Prep Shooter", m_shooter.runShooterFactory());
        NamedCommands.registerCommand("Shoot", new WaitUntilCommand(m_shooter::isShooterAtFullSpeed)
                .andThen(m_intake.shootIndexerFactory().withTimeout(0.3))
                .unless(() -> !m_intake.hasNoteInIndexer() && !m_intake.sawNote()));
        NamedCommands.registerCommand("Shoot Soon", new WaitUntilCommand(m_shooter::isShooterAtAlmostFullSpeed)
                .andThen(m_intake.shootIndexerFactory().withTimeout(0.3)));
        NamedCommands.registerCommand("Wait For Shooter", new WaitUntilCommand(m_shooter::isShooterAtFullSpeed));
        NamedCommands.registerCommand("Localize", drivetrain.localizeFactory());
        NamedCommands.registerCommand("Intake", m_intake.intakeFactory().withTimeout(4.0));
        NamedCommands.registerCommand("Pivot Stow", m_elevator.stowFactory());
        // NamedCommands.registerCommand("Pivot Aim",
        // m_elevator.goToAnlgeFactory(p_autoCloseAim.getValue())
        // .until(() -> m_elevator.pivotOnTarget(p_autoCloseAim.getValue(), 2)));
        NamedCommands.registerCommand("Pivot Aim",
                m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
                        .until(() -> m_elevator.pivotOnTarget(m_aiming.speakerAngleForShooter(),
                                2.0))
                        .unless(() -> !m_intake.hasNoteInIndexer() && !m_intake.sawNote()));
        NamedCommands.registerCommand("Pivot Active Aim",
                m_elevator.goToAimingPosition(() -> m_aiming.odomSpeakerAngle(drivetrain.getPose())));
        NamedCommands.registerCommand("Pivot Aim Minus 4",
                m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter() - 3.5)
                        .until(() -> m_elevator.pivotOnTarget(m_aiming.speakerAngleForShooter() - 3.5,
                                2.0)));
        NamedCommands.registerCommand("Aim",
                new ParallelCommandGroup(m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
                        .until(() -> m_elevator.pivotOnTarget(m_aiming.speakerAngleForShooter(), 2.0)),
                        drivetrain.applyRequest(drivetrain.autoSnapToAngleRequest()),
                        drivetrain.aimAtSpeakerFactory().until(drivetrain::onTarget)));
        NamedCommands.registerCommand("Stop Shooter", m_shooter.stopShooterFactory().withTimeout(0.2));
        NamedCommands.registerCommand("Pivot Calibrated",
                new WaitUntilCommand(m_elevator::isPivotCalibrated));

        configureSmartDashboardButtons();

        // set default commands
        // set below in telop init
        // drivetrain.setDefaultCommand(drivetrain.defaultDriveCommand(joystick));
        drivetrain.register();
        drivetrain.resetPose(new Pose2d());

        m_shooter.setDefaultCommand(
                m_shooter.stopShooterFactory().unless(drivetrain.tipping()));
        m_intake.setDefaultCommand(m_intake.stopMovingFactory().unless(drivetrain.tipping()));
        m_elevator.setDefaultCommand(m_elevator.stowFactory().unless(drivetrain.tipping()));
        m_climber.setDefaultCommand(m_climber.stowArmFactory().unless(drivetrain.tipping()));
    }

    private void configureRosNetworkTablesBridge() {
        NetworkTableInstance instance = NetworkTableInstance.create();
        instance.startClient3("coprocessor");
        instance.setServer("10.0.88.44", 5800);

        ROSNetworkTablesBridge bridge = new ROSNetworkTablesBridge(instance.getTable(""), 20);
        tfListenerCompact = new TFListenerCompact(bridge, "/tf_compact");
        TagSubscriber tagsub = new TagSubscriber(bridge);
        BridgePublisher<MarkerArray> aimPub = new BridgePublisher<>(bridge, "target_aiming");
        coprocessorBridge = new CoprocessorBridge(drivetrain, bridge, tfListenerCompact);
        bagManager = new BagManager(bridge);

        m_aiming.setTFListener(tfListenerCompact);
        m_aiming.setTagListener(tagsub);
        m_aiming.setAimPub(aimPub);

        m_lights = new Lights(drivetrain, m_intake,
                m_elevator,
                m_shooter, m_climber,
                coprocessorBridge, m_aiming, () -> m_autoCommandName);

    }

    private void configureDriverController() {
        joystick.b().onTrue(drivetrain.setHeadingFactory(270));
        joystick.x().onTrue(drivetrain.setHeadingFactory(90));
        joystick.y().onTrue(drivetrain.setHeadingFactory(0));
        joystick.a().whileTrue(drivetrain.aimAtHeadingFactory(joystickAimAtHeading()));

        // joystick.rightTrigger()
        // .onTrue(drivetrain.localizeFactory().unless(m_elevator::isElevatorNotDown)).debounce(0.25);
        joystick.rightTrigger().onTrue(m_intake.shootIndexerFactory()
                .unless(() -> drivetrain.tipping().getAsBoolean() || !m_intake.hasNoteInIndexer())
                .until(() -> !m_intake.hasNoteInIndexer())
                .andThen(m_intake.intakeFactory()));
        joystick.rightBumper()
                .whileTrue(
                        m_shooter.runShooterFactory().alongWith(new WaitUntilCommand(m_shooter::isShooterAtFullSpeed))
                                .andThen(setRumble()).unless(drivetrain.tipping()))
                .whileTrue(drivetrain.aimAtSpeakerFactory().unless(drivetrain.tipping()))
                .whileTrue(m_elevator.goToAimingPosition(() -> m_aiming.speakerAngleForShooter())
                        .unless(() -> drivetrain.tipping().getAsBoolean() || !m_intake.hasNoteInIndexer()))
                .onTrue(m_lights.setShootingFactory(true))
                .onFalse(m_lights.setShootingFactory(false));
        joystick.leftBumper()
                .whileTrue(drivetrain.aimAtAmpDumpingGroundFactory(buttonBox.button(17))
                        .alongWith(m_elevator.setFlatFactory())
                        .alongWith(m_shooter.runShuttlePassFactory(buttonBox.button(17))))
                .onFalse(new InstantCommand(() -> m_shooter.m_shuttlePass = false));
        joystick.leftTrigger().whileTrue(m_shooter.runShooterFactory());
    }

    private void configureButtonBox() {
        buttonBox.button(10).whileTrue(m_intake.intakeNoSawNoteFactory().unless(drivetrain.tipping()));
        buttonBox.button(20)
                .whileTrue(m_intake.shootIndexerFactory().unless(drivetrain.tipping()));
        buttonBox.button(18).whileTrue(m_intake.rejectFactory().unless(drivetrain.tipping()));
        buttonBox.button(5)
                .whileTrue(m_elevator.setPodiumFactory().unless(drivetrain.tipping()));
        buttonBox.button(6)
                .whileTrue(m_shooter.slowSpeedFactory().until(() -> m_shooter.isShooterAtSlowSpeed())
                        .andThen(m_elevator.setAmpFactory())
                        .until(() -> m_elevator.pivotOnTargetForAmp() && m_elevator.elevatorOnTarget())
                        .andThen(m_shooter.runAmpTrapSpeedFactory()).unless(drivetrain.tipping()))
                .onFalse(m_elevator.elevatorDownFactory()
                        .until(() -> m_elevator.elevatorOnTarget() && m_elevator.pivotOnTarget(42.0, 2.0))
                        .unless(drivetrain.tipping()))
                .onFalse(m_shooter.stopShooterFactory().unless(drivetrain.tipping()));
        buttonBox.button(11)
                .onTrue(m_elevator.stowFactory()
                        .until(() -> m_elevator.areElevatorAndPivotDown())
                        .andThen(m_climber.stowArmFactory())
                        .unless(drivetrain.tipping()));
        buttonBox.button(2).onTrue(m_climber.prepArmsFactory().alongWith(m_elevator.stowFactory())
                .unless(drivetrain.tipping()))
                .negate().and(m_climber::isPrepped)
                .onTrue(m_climber.readjustArmsFactory().alongWith(m_elevator.stowFactory()));
        buttonBox.button(15)
                .onTrue(climb(false));
        buttonBox.button(19).onTrue(m_climber.softLandingFactory().alongWith(m_elevator.climbFactory())
                .unless(drivetrain.tipping()))
                .onFalse(new InstantCommand(() -> m_intake.enableAutoMode()));
        buttonBox.button(8)
                .onTrue(new InstantCommand(() -> m_intake.disableAutoMode()).andThen(new WaitCommand(0.1))
                        .andThen(m_shooter.slowSpeedFactory().until(
                                () -> m_shooter.isShooterAtSlowSpeed()))
                        .andThen(new ParallelCommandGroup(climb(true),
                                new WaitUntilCommand(
                                        () -> m_elevator.pivotOnTargetForAmp() && m_elevator.isElevatorUp())
                                        .andThen(m_shooter.runAmpTrapSpeedFactory().withTimeout(1.5))
                                        .andThen(m_intake.shootIndexerFactory())))
                        .unless(drivetrain.tipping()));
        buttonBox.button(16).whileTrue(intakeFromSource())
                .onTrue(m_lights.setYumYumIntakeFactory(true))
                .onFalse(m_lights.setYumYumIntakeFactory(false))
                .onFalse(new InstantCommand(m_intake::enableAutoMode).andThen(m_intake.intakeFactory()));
        buttonBox.button(21).whileTrue(new InstantCommand(m_intake::disableAutoMode).andThen(goblinModeFactory()))
                .onFalse(new InstantCommand(m_intake::enableAutoMode));
        buttonBox.button(12)
                .whileTrue(drivetrain.localizeFactory().andThen(drivetrain.pathFindingCommand("Amp"))
                        .andThen(drivetrain.setHeadingFactory(() -> drivetrain.getCurrentRobotAngle())));
        // buttonBox.button(16).whileTrue(m_elevator.goToAimingPosition(() ->
        // m_aiming.speakerAngleForShooter()));
        buttonBox.button(13)
                .whileTrue(drivetrain.localizeFactory().andThen(drivetrain.pathFindingCommand("StageLeft"))
                        .andThen(drivetrain.setHeadingFactory(() -> drivetrain.getCurrentRobotAngle())));
    }

    private void configureSmartDashboardButtons() {
        // Drive
        SmartDashboard.putData("SetLowPowerMode", drivetrain.lowPowerModeFactory());
        SmartDashboard.putData("SetHighPowerMode", drivetrain.highPowerModeFactory());
        SmartDashboard.putData("CalibrateDrivetrain", drivetrain.calibrateFactory().ignoringDisable(true));
        SmartDashboard.putData("Localize", drivetrain.localizeFactory().ignoringDisable(true));

        // Shooter
        // SmartDashboard.putData("Run Shooter", m_shooter.runShooterCommand());
        // SmartDashboard.putData("Stop Shooter", m_shooter.stopShooterCommand());

        // Lights
        SmartDashboard.putData("TieDye",
                m_lights.tieDyeFactory().ignoringDisable(true));
        SmartDashboard.putData("fire",
                m_lights.setFireFactory().ignoringDisable(true));
        SmartDashboard.putData("set red", m_lights.setLEDFactory(255, 0, 0).ignoringDisable(true));
        SmartDashboard.putData("set green", m_lights.setLEDFactory(0, 255, 0).ignoringDisable(true));
        SmartDashboard.putData("set blue", m_lights.setLEDFactory(0, 0, 255).ignoringDisable(true));

        // Elevator
        SmartDashboard.putData("Calibrate Pivot", m_elevator.calibratePivotFactory());
        SmartDashboard.putData("Calibrate Elevator", m_elevator.calibrateElevatorFactory());
        SmartDashboard.putData("Go to Stow", m_elevator.stowFactory());
        SmartDashboard.putData("Go To Flat", m_elevator.setFlatFactory());

        // Climber
        SmartDashboard.putData("ClimberCalibrate", m_climber.calibrateFactory());
        SmartDashboard.putData("ClimberCoastMode", m_climber.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ClimberBrakeMode", m_climber.enableBrakeModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorCoastMode", m_elevator.enableCoastModeFactory().ignoringDisable(true));
        SmartDashboard.putData("ElevatorBrakeMode", m_elevator.enableBrakeModeFactory().ignoringDisable(true));

        // Test
        SmartDashboard.putData("Rumble", setRumble().ignoringDisable(true));
    }

    private void configureBindings() {
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Trigger isRightStickZero() {
        return new Trigger(() -> RobotState.isTeleop() && Math.abs(joystick.getRightX()) < 0.01
                && Math.abs(joystick.getRightY()) < 0.01);
    }

    public void teleopInit() {
        if (coasting) {
            // brake
            m_climber.enableBrakeMode();
            m_elevator.enableBrakeMode();
        }
        // enable triggers
        m_intake.hasNote()
                .onTrue((m_shooter.runIdleSpeedFactory())
                        .unless(() -> !m_intake.m_automaticMode || m_shooter.m_shuttlePass))
                .onTrue(setRumble().unless(() -> !m_intake.m_automaticMode)).onTrue(m_lights.spinLeftFactory());
        m_intake.hasNote().and(() -> !m_intake.m_automaticMode)
                .onFalse(m_intake.intakeFactory()
                        .alongWith(m_shooter.stopShooterFactory().unless(() -> m_shooter.m_shuttlePass)))
                .debounce(0.25);

        m_aiming.isInWing().whileTrue(m_shooter.runShooterFactory());
        drivetrain.tipping().whileTrue(m_climber.holdPositionFactory()).whileTrue(m_elevator.holdPositionFactory());
        m_shooter.shooterAtSpeed().onTrue(setRumble());

        isRightStickZero().debounce(0.25, DebounceType.kRising)
                .onTrue(drivetrain.setHeadingFactory(() -> drivetrain.getState().Pose.getRotation().getDegrees()))
                .whileFalse(drivetrain.applyRequest(drivetrain.fieldCentricRequest(joystick)));

        if (!isRightStickZero().getAsBoolean()) {
            drivetrain.applyRequest(drivetrain.fieldCentricRequest(joystick))
                    .until(isRightStickZero().debounce(0.25, DebounceType.kRising)).schedule();
        }

        drivetrain.setTargetHeading(drivetrain.getState().Pose.getRotation().getDegrees());
        drivetrain.setDefaultCommand(drivetrain.defaultDriveCommand(joystick));

        m_intake.intakeFactory().schedule();
    }

    public void disabledInit() {
        indexerStart = m_intake.getIndexerPosition();
        readyToCoast = coasting = false;
    }

    public void disabledPeriodic() {
        detectCoastGesture();

        String nextAuto = m_autoCommandName;
        if (buttonBox.button(12).getAsBoolean() && !nextAuto.equals("Nutrons")) {
            m_autoCommand = drivetrain.getAutoPath("Nutrons");
            nextAuto = "Nutrons";
        }

        if (buttonBox.button(13).getAsBoolean() && !nextAuto.equals("Nutrons2")) {
            m_autoCommand = drivetrain.getAutoPath("Nutrons2");
            nextAuto = "Nutrons2";
        }

        if (buttonBox.button(6).getAsBoolean() && !m_autoCommandName.equals("FivePiece")) {
            m_autoCommand = drivetrain.getAutoPath("FivePiece");
            nextAuto = "FivePiece";
        }

        if (buttonBox.button(10).getAsBoolean() && !nextAuto.equals("EiffelTower")) {
            m_autoCommand = drivetrain.getAutoPath("EiffelTower");
            nextAuto = "EiffelTower";
        }

        if (buttonBox.button(11).getAsBoolean() && !nextAuto.equals("Cleanside")) {
            m_autoCommand = drivetrain.getAutoPath("Cleanside");
            nextAuto = "Cleanside";
        }

        if (buttonBox.button(19).getAsBoolean() && !nextAuto.equals("Cleanside2")) {
            m_autoCommand = drivetrain.getAutoPath("Cleanside2");
            nextAuto = "Cleanside2";
        }

        if ((buttonBox.button(8)).getAsBoolean() && !m_autoCommandName.equals("SixPiece")) {
            m_autoCommand = drivetrain.getAutoPath("SixPiece");
            nextAuto = "SixPiece";
        }

        if (buttonBox.button(20).getAsBoolean()) {
            m_autoCommand = new WaitCommand(15);
            nextAuto = "Waiting";
        }

        if (!nextAuto.equals(m_autoCommandName)) {
            bagManager.startBag(); // Start recording
            m_autoCommandName = nextAuto;
        }

        SmartDashboard.putString("Auto", m_autoCommandName);
    }

    private void detectCoastGesture() {
        boolean hasNote = m_intake.hasNoteInIndexer();

        if (!hasNote && !readyToCoast && !coasting && m_intake.getIndexerPosition() - indexerStart < -4.0) {
            readyToCoast = true;
            m_lights.setLED(0, 0, 255);
        }

        if (readyToCoast && !hasNote && m_intake.getIndexerPosition() - indexerStart > 0.0) {
            // coast
            m_lights.setLED(255, 0, 0);
            m_climber.enableCoastMode();
            m_elevator.enableCoastMode();
            readyToCoast = false;
            coasting = true;
        }

        if (coasting && hasNote) {
            // brake
            // NOTE: Be sure to enable brake mode in teleopInit above!
            m_lights.disableLED();
            m_climber.enableBrakeMode();
            m_elevator.enableBrakeMode();
            indexerStart = m_intake.getIndexerPosition();
            coasting = false;
        }

        if (readyToCoast && hasNote) {
            m_lights.disableLED();
            indexerStart = m_intake.getIndexerPosition();
            readyToCoast = false;
        }

        if (!coasting && !readyToCoast) {
            m_climber.enableBrakeMode();
            m_elevator.enableBrakeMode();
        }

    }

    private Command setRumble() {
        return new SequentialCommandGroup(new InstantCommand(
                () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                new WaitCommand(1),
                new InstantCommand(() -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    }

    public void autonomousInit() {
        m_elevator.calibratePivot();
        // drivetrain.localize();
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}
