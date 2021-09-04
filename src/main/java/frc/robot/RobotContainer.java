/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AccurateFeed;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.commands.RunShooterPreset.ShooterPreset;
import frc.robot.commands.FeedUnstick;
import frc.robot.commands.HoldPosition;
import frc.robot.commands.IdleLimelightControl;
import frc.robot.commands.LimelightOdometry;
import frc.robot.commands.LimelightTest;
import frc.robot.commands.PointAtTargetAndShoot;
import frc.robot.commands.RunAutoNavBarrelRacing;
import frc.robot.commands.RunAutoNavBounce;
import frc.robot.commands.RunAutoNavSlalom;
import frc.robot.commands.PointAtTargetAndShootTrenchRun;
import frc.robot.commands.PointAtTargetWithOdometry;
import frc.robot.commands.PowerPortChallengeAuto;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunGalacticSearchABlue;
import frc.robot.commands.RunGalacticSearchARed;
import frc.robot.commands.RunGalacticSearchBBlue;
import frc.robot.commands.RunGalacticSearchBRed;
import frc.robot.commands.RunGalacticSearchVision;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunHyperdriveLightspeedCircuit;
import frc.robot.commands.RunIntakeBackwards;
import frc.robot.commands.RunIntakeForwards;
import frc.robot.commands.RunMotionProfile;
import frc.robot.commands.RunShooterAtDistance;
import frc.robot.commands.RunShooterPreset;
import frc.robot.commands.RunShooterRoller;
import frc.robot.commands.SetLEDOverride;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VelocityPIDTuner;
import frc.robot.oi.DummyOI;
import frc.robot.oi.IDriverOI;
import frc.robot.oi.IDriverOverrideOI;
import frc.robot.oi.IOperatorOI;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.oi.OIArduinoConsole;
import frc.robot.oi.OIDualJoysticks;
import frc.robot.oi.OIHandheld;
import frc.robot.oi.OIHandheldAllInOne;
import frc.robot.oi.OIHandheldWithOverrides;
import frc.robot.oi.OIOperatorHandheld;
import frc.robot.oi.OIeStopConsole;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.LimelightInterface.LimelightStreamingMode;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.drive.CTREDriveTrain;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;
import frc.robot.subsystems.drive.SparkMAXDriveTrain;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.PressureSensor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double navXWaitTime = 5; // Maximum number of seconds to wait for the navX to initialize
  private static final Pose2d initialAutoPosition = new Pose2d(Constants.fieldLength - Constants.initiationLine - 16,
      -Constants.visionTargetHorizDist, Rotation2d.fromDegrees(0));

  private static final DummyOI dummyOI = new DummyOI();
  private IDriverOI driverOI = dummyOI;
  private IDriverOverrideOI driverOverrideOI = dummyOI;
  private IOperatorOI operatorOI = dummyOI;
  private Alert driverOIAlert = new Alert("No driver controller.", AlertType.WARNING);
  private Alert driverOverrideOIAlert = new Alert("No override controller.", AlertType.WARNING);
  private Alert operatorOIAlert = new Alert("No operator controller.", AlertType.WARNING);
  private String[] lastJoystickNames;
  private boolean changedToCoast;

  // The robot's subsystems and commands are defined here...
  private final CameraSystem cameraSubsystem = new CameraSystem();
  private final LimelightInterface limelight = new LimelightInterface();
  private DriveTrainBase driveSubsystem;
  private final ShooterFlyWheel shooterFlyWheel = new ShooterFlyWheel((led, state) -> operatorOI.updateLED(led, state),
      (double rpm) -> operatorOI.setFlyWheelSpeed(rpm));
  private final ShooterRoller shooterRoller = new ShooterRoller();
  private final PressureSensor pressureSensor = new PressureSensor(0, (pressure) -> operatorOI.setPressure(pressure));
  private final ShooterHood shooterHood = new ShooterHood(pressureSensor,
      (led, state) -> operatorOI.updateLED(led, state), (position) -> operatorOI.setHoodPosition(position));
  private final Intake intake = new Intake((led, state) -> operatorOI.updateLED(led, state));
  private final Hopper hopper = new Hopper();
  private final Climber climber = new Climber();
  private RobotOdometry odometry;

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private SendableChooser<JoystickMode> joystickModeChooser;
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final SendableChooser<ShooterPreset> shootingPresetChooser = new SendableChooser<ShooterPreset>();
  private final SendableChooser<FeedMode> feedModeChooser = new SendableChooser<FeedMode>();
  private final SendableChooser<Double> speedLimitChooser = new SendableChooser<Double>();

  private final RunGalacticSearchVision galacticSearchCommand;

  private LimelightOdometry limelightOdometry;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.", AlertType.INFO).set(true);
    }

    // The subsystems can't be recreated when OI changes so provide them with a
    // BooleanSupplier to access the current value from whatever OI is current
    BooleanSupplier openLoopSwitchAccess = () -> driverOverrideOI.getOpenLoopSwitch().get();
    BooleanSupplier driveDisableSwitchAccess = () -> driverOverrideOI.getDriveDisableSwitch().get();
    BooleanSupplier shiftLockSwitchAccess = () -> driverOverrideOI.getShiftLockSwitch().get();
    switch (Constants.getRobot()) {
      case ROBOT_2020:
      case ROBOT_2020_DRIVE:
        driveSubsystem = new SparkMAXDriveTrain(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
        break;
      case ROBOT_2019:
      case ORIGINAL_ROBOT_2018:
      case REBOT:
      case NOTBOT:
        driveSubsystem = new CTREDriveTrain(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
        break;
    }
    // Odometry must be instantiated after drive and AHRS and after the NavX
    // initializes
    Timer navXTimer = new Timer();
    navXTimer.start();
    while (ahrs.getByteCount() == 0 && navXTimer.get() <= navXWaitTime) {
      Timer.delay(0.01);
    }
    if (navXTimer.get() >= navXWaitTime) {
      new Alert("Failed to initialize navX, odometry will likely be nonfunctional.", AlertType.ERROR).set(true);
    }
    odometry = new RobotOdometry(driveSubsystem, ahrs);
    limelightOdometry = new LimelightOdometry(limelight, odometry);
    odometry.setDefaultCommand(limelightOdometry);
    limelight.setDefaultCommand(new IdleLimelightControl(limelight,
        () -> driverOverrideOI.getLimelightLEDDisableSwitch().get() || shootingPresetChooser.getSelected() != null));
    limelight.setStreamingMode(LimelightStreamingMode.PIP_SECONDARY);
    galacticSearchCommand = new RunGalacticSearchVision(odometry, driveSubsystem, intake);

    updateOIType();

    autoChooser.setDefaultOption("Do Nothing", null);
    if (Constants.tuningMode) {
      autoChooser.addOption("Turn 90 degrees", new TurnToAngle(driveSubsystem, ahrs, 90));
      autoChooser.addOption("Turn 15 degrees", new TurnToAngle(driveSubsystem, ahrs, 15));
      autoChooser.addOption("Turn about 135 degrees", new TurnToAngle(driveSubsystem, ahrs, 135, true, 5));
      autoChooser.addOption("Drive 5 feet", new DriveDistanceOnHeading(driveSubsystem, ahrs, 60));
      autoChooser.addOption("Drive velocity", new VelocityPIDTuner(driveSubsystem));
      autoChooser.addOption("Drive 5 feet (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(0, 60, new Rotation2d(0)), 0, false, true));
      autoChooser.addOption("Drive to 5 feet absolute (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(0, 60, new Rotation2d(0)), 0, false, false));
      autoChooser.addOption("Drive 5 foot arc (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(180, 60, Rotation2d.fromDegrees(90)), 0, false, true));
    }
    autoChooser.addOption("Fire loaded balls",
        new PointAtTargetAndShoot(driveSubsystem, odometry, limelight, ahrs, hopper, shooterRoller, shooterFlyWheel,
            shooterHood, pressureSensor, (led, state) -> operatorOI.updateLED(led, state),
            (position) -> operatorOI.setHoodPosition(position)));
    autoChooser.addOption("Fire loaded balls & collect trench run",
        new PointAtTargetAndShootTrenchRun(driveSubsystem, odometry, limelight, ahrs, hopper, shooterRoller,
            shooterFlyWheel, shooterHood, intake, pressureSensor, (led, state) -> operatorOI.updateLED(led, state),
            (position) -> operatorOI.setHoodPosition(position)));
    autoChooser.addOption("Galactic Search (Vision)", galacticSearchCommand);
    autoChooser.addOption("Galactic Search (A/Blue)", new RunGalacticSearchABlue(odometry, driveSubsystem, intake));
    autoChooser.addOption("Galactic Search (A/Red)", new RunGalacticSearchARed(odometry, driveSubsystem, intake));
    autoChooser.addOption("Galactic Search (B/Blue)", new RunGalacticSearchBBlue(odometry, driveSubsystem, intake));
    autoChooser.addOption("Galactic Search (B/Red)", new RunGalacticSearchBRed(odometry, driveSubsystem, intake));
    autoChooser.addOption("AutoNav (Barrel Racing)", new RunAutoNavBarrelRacing(odometry, driveSubsystem));
    autoChooser.addOption("AutoNav (Slalom)", new RunAutoNavSlalom(odometry, driveSubsystem));
    autoChooser.addOption("AutoNav (Bounce)", new RunAutoNavBounce(odometry, driveSubsystem, intake));
    autoChooser.addOption("Hyperdrive (Lightspeed Circuit)",
        new RunHyperdriveLightspeedCircuit(odometry, driveSubsystem));
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Set up demo choosers
    shootingPresetChooser.setDefaultOption("--Competition Mode--", null);
    shootingPresetChooser.addOption("Demo: Long Shot", new ShooterPreset(HoodPosition.TRENCH, 6000));
    shootingPresetChooser.addOption("Demo: Play Long Catch", new ShooterPreset(HoodPosition.FRONT_LINE, 5000));
    shootingPresetChooser.addOption("Demo: Play Short Catch", new ShooterPreset(HoodPosition.FRONT_LINE, 3000));
    shootingPresetChooser.addOption("Demo: Tall Shot", new ShooterPreset(HoodPosition.WALL, 6000));
    SmartDashboard.putData("Demo/Shooting Preset", shootingPresetChooser);

    feedModeChooser.setDefaultOption("--Competition Mode--", FeedMode.NORMAL);
    feedModeChooser.addOption("Demo (Automatic)", FeedMode.DEMO);
    feedModeChooser.addOption("Interstellar Accuracy", FeedMode.ACCURACY);
    SmartDashboard.putData("Demo/Feed Mode", feedModeChooser);

    speedLimitChooser.setDefaultOption("--Competition Mode--", 1.0);
    speedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    speedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    speedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    SmartDashboard.putData("Demo/Speed Limit", speedLimitChooser);
  }

  public void updateOIType() {
    String[] joystickNames = { null, null, null, null, null, null };
    int joystickNum;
    for (joystickNum = 0; joystickNum < 6; joystickNum++) {
      joystickNames[joystickNum] = new Joystick(joystickNum).getName();
    }
    if (!Arrays.equals(joystickNames, lastJoystickNames)) {
      // Button mapping must be cleared before instantiating new OI because the new OI
      // might need to map buttons internally
      CommandScheduler.getInstance().clearButtons();

      // Reset to dummy OI to guarantee old controls won't carry over
      driverOI = dummyOI;
      driverOverrideOI = dummyOI;
      operatorOI = dummyOI;

      // The first name that is seen will be used and any other
      // controller names will be ignored (will only complete a pair)
      int firstController = 0; // Used to store first ID for controller pairs
      String firstControllerName = null;
      String joystickName;

      // Look for operator controller
      for (joystickNum = 0; joystickNum < 6; joystickNum++) {
        joystickName = joystickNames[joystickNum];
        switch (joystickName) {
          case "Arduino Leonardo":
            if (firstControllerName == null) {
              firstController = joystickNum;
              firstControllerName = joystickName;
            } else if (firstControllerName.equals(joystickName)) {
              OIArduinoConsole arduinoConsole = new OIArduinoConsole(firstController, joystickNum);
              operatorOI = arduinoConsole;
              driverOverrideOI = arduinoConsole;
              System.out.println("Operator: Arduino console");
            }
            break;
          case "eStop Robotics HID":
            if (firstControllerName == null) {
              firstController = joystickNum;
              firstControllerName = joystickName;
            } else if (firstControllerName.equals(joystickName)) {
              OIeStopConsole eStopConsole = new OIeStopConsole(firstController, joystickNum);
              operatorOI = eStopConsole;
              driverOverrideOI = eStopConsole;
              System.out.println("Operator: eStop console");
            }
            break;
        }
      }

      // Look for driver controller
      firstControllerName = null;
      boolean xboxDriver = false;
      boolean xboxOperator = false;
      for (joystickNum = 0; joystickNum < 6; joystickNum++) {
        joystickName = joystickNames[joystickNum];
        switch (joystickName) {
          case "Controller (XBOX 360 For Windows)":
          case "Controller (Gamepad F310)":
          case "Controller (Gamepad for Xbox 360)":
          case "Controller (Afterglow Gamepad for Xbox 360)":
          case "Controller (Xbox One For Windows)":
          case "XBOX 360 For Windows (Controller)":
          case "Gamepad F310 (Controller)":
          case "Gamepad for Xbox 360 (Controller)":
          case "Afterglow Gamepad for Xbox 360 (Controller)":
          case "Xbox One For Windows (Controller)":
            if (firstControllerName == null) {
              firstControllerName = joystickName;
              firstController = joystickNum;
              xboxDriver = true;
            } else if (operatorOI.equals(dummyOI)) {
              operatorOI = new OIOperatorHandheld(joystickNum);
              xboxOperator = true;
              System.out.println("Operator: XBox/F310 controller");
            }
            break;
          case "Logitech Attack 3":
            if (firstControllerName == null) {
              firstController = joystickNum;
              firstControllerName = joystickName;
            } else if (firstControllerName.equals(joystickName)) {
              driverOI = new OIDualJoysticks(firstController, joystickNum);
              System.out.println("Driver: Dual Attack 3");
            }
            break;
        }
      }
      if (xboxDriver) {
        if (!operatorOI.equals(dummyOI) && !xboxOperator) {
          driverOI = new OIHandheld(firstController);
          System.out.println("Driver: XBox/F310 controller");
        } else if (!operatorOI.equals(dummyOI)) {
          OIHandheldWithOverrides driverWithOverrides = new OIHandheldWithOverrides(firstController);
          driverOI = driverWithOverrides;
          driverOverrideOI = driverWithOverrides;
          System.out.println("Driver: XBox/F310 controller w/ overrides");
        } else {
          OIHandheldAllInOne handheldAllInOne = new OIHandheldAllInOne(firstController);
          driverOI = handheldAllInOne;
          operatorOI = handheldAllInOne;
          driverOverrideOI = handheldAllInOne;
          System.out.println("Driver/operator: XBox/F310 controller");
        }
      }

      driverOIAlert.set(driverOI.equals(dummyOI));
      driverOverrideOIAlert.set(driverOverrideOI.equals(dummyOI));
      operatorOIAlert.set(operatorOI.equals(dummyOI));

      lastJoystickNames = joystickNames;
      configureInputs();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureInputs() {
    setupJoystickModeChooser();

    DriveWithJoysticks driveCommand = new DriveWithJoysticks(driverOI::getLeftDriveX, driverOI::getLeftDriveY,
        driverOI::getLeftDriveTrigger, driverOI::getRightDriveX, driverOI::getRightDriveY,
        driverOI::getRightDriveTrigger, driverOI::getQuickTurn, driverOI::getDeadband, driverOI::getSniperMode,
        driverOI::getSniperLevel, driverOI::getSniperHighLevel, driverOI::getSniperLowLevel, driverOI::getSniperLow,
        driverOI::getSniperHigh, driverOI.hasDualSniperMode(), joystickModeChooser, speedLimitChooser, driveSubsystem);
    driveSubsystem.setDefaultCommand(driveCommand);
    driverOI.getJoysticksForwardButton().whenActive(() -> driveCommand.setReversed(false));
    driverOI.getJoysticksReverseButton().whenActive(() -> driveCommand.setReversed(true));

    driverOverrideOI.getOpenLoopSwitch()
        .whenActive(new SetLEDOverride(OILED.OPEN_LOOP, OILEDState.ON, operatorOI::updateLED));
    driverOverrideOI.getOpenLoopSwitch()
        .whenInactive(new SetLEDOverride(OILED.OPEN_LOOP, OILEDState.OFF, operatorOI::updateLED));
    operatorOI.updateLED(OILED.OPEN_LOOP, driverOverrideOI.getOpenLoopSwitch().get() ? OILEDState.ON : OILEDState.OFF);

    driverOverrideOI.getDriveDisableSwitch()
        .whenActive(new SetLEDOverride(OILED.DRIVE_DISABLE, OILEDState.ON, operatorOI::updateLED));
    driverOverrideOI.getDriveDisableSwitch()
        .whenInactive(new SetLEDOverride(OILED.DRIVE_DISABLE, OILEDState.OFF, operatorOI::updateLED));
    operatorOI.updateLED(OILED.DRIVE_DISABLE,
        driverOverrideOI.getDriveDisableSwitch().get() ? OILEDState.ON : OILEDState.OFF);
    // The DriveTrain will enforce the switches but this makes sure they are applied
    // immediately
    // neutralOutput is safer than stop since it prevents the motors from running
    driverOverrideOI.getDriveDisableSwitch().whenActive(driveSubsystem::neutralOutput, driveSubsystem);
    // This prevents the drive train from running in the old mode and since behavior
    // changes anyway stopping if the current command isn't calling drive is
    // reasonable. A brief neutralOutput while driving won't cause a noticable
    // change anyway
    driverOverrideOI.getOpenLoopSwitch().whenActive(driveSubsystem::neutralOutput);
    driverOverrideOI.getOpenLoopSwitch().whenInactive(driveSubsystem::neutralOutput);

    driverOI.getHighGearButton().whenActive(() -> driveSubsystem.switchGear(DriveGear.HIGH), driveSubsystem);
    driverOI.getLowGearButton().whenActive(() -> driveSubsystem.switchGear(DriveGear.LOW), driveSubsystem);
    driverOI.getToggleGearButton().whenActive(() -> driveSubsystem.switchGear(driveSubsystem.getCurrentGear().invert()),
        driveSubsystem);

    // Since useFrontCamera/useSecondCamera don't need arguments they can be passed
    // directly to whenActive
    driverOI.getFrontCameraButton().whenActive(cameraSubsystem::useFrontCamera, cameraSubsystem);
    driverOI.getSecondCameraButton().whenActive(cameraSubsystem::useSecondCamera, cameraSubsystem);

    driverOI.getVisionTestButton().whenActive(new LimelightTest(limelight, ahrs));

    driverOI.getShooterRollerButton().and(new Trigger(() -> feedModeChooser.getSelected() == FeedMode.NORMAL))
        .and(new Trigger(shooterHood::atTargetPosition)).and(new Trigger(shooterFlyWheel::safeToFeed))
        .whileActiveContinuous(
            new RunHopper(hopper).alongWith(new RunShooterRoller(shooterRoller), new HoldPosition(driveSubsystem)));
    driverOI.getShooterRollerButton().and(new Trigger(() -> feedModeChooser.getSelected() == FeedMode.DEMO))
        .and(new Trigger(shooterHood::atTargetPosition)).and(new Trigger(shooterFlyWheel::atSetpoint))
        .whileActiveContinuous(new RunHopper(hopper).alongWith(new RunShooterRoller(shooterRoller)));
    driverOI.getShooterRollerButton().and(new Trigger(() -> feedModeChooser.getSelected() == FeedMode.ACCURACY))
        .whileActiveContinuous(new AccurateFeed(shooterRoller, hopper, shooterFlyWheel, shooterHood)
            .alongWith(new HoldPosition(driveSubsystem)));
    driverOI.getShooterUnstickButton()
        .whileActiveContinuous(new FeedUnstick(shooterRoller, hopper, operatorOI::updateLED));

    operatorOI.getIntakeExtendButton().whenActive(intake::extend, intake);
    operatorOI.getIntakeRetractButton().whenActive(intake::retract, intake);

    RunIntakeForwards runIntakeForwards = new RunIntakeForwards(intake);
    RunIntakeBackwards runIntakeBackwards = new RunIntakeBackwards(intake);
    operatorOI.getRunIntakeForwardsButton().whileActiveContinuous(runIntakeForwards);
    operatorOI.getRunIntakeBackwardsButton().whileActiveContinuous(runIntakeBackwards);

    intake.updateLEDs();

    // Which shooting mode? Only one should be active at a time.
    Trigger manualHood = new Trigger(() -> shootingPresetChooser.getSelected() == null)
        .and(operatorOI.getManualHoodSwitch());
    Trigger autoHood = new Trigger(() -> shootingPresetChooser.getSelected() == null)
        .and(operatorOI.getManualHoodSwitch().negate());
    Trigger demoPreset = new Trigger(() -> shootingPresetChooser.getSelected() == null).negate();

    Trigger demoFeed = new Trigger(() -> feedModeChooser.getSelected() == FeedMode.DEMO);
    Command runShooterAutoHood = new RunShooterAtDistance(shooterFlyWheel, shooterHood, odometry, true);
    Command runShooterManualHood = new RunShooterAtDistance(shooterFlyWheel, shooterHood, odometry, false);
    Command runShooterPreset = new RunShooterPreset(shooterFlyWheel, shooterHood, shootingPresetChooser);
    operatorOI.getShooterFlywheelRunButton().and(manualHood).and(demoFeed.negate()).whenActive(runShooterManualHood);
    operatorOI.getShooterFlywheelRunButton().and(autoHood).and(demoFeed.negate()).whenActive(runShooterAutoHood);
    operatorOI.getShooterFlywheelRunButton().and(demoPreset).and(demoFeed.negate()).whenActive(runShooterPreset);

    driverOI.getShooterRollerButton().and(manualHood).and(demoFeed).whileActiveContinuous(runShooterManualHood);
    driverOI.getShooterRollerButton().and(autoHood).and(demoFeed).whileActiveContinuous(runShooterAutoHood);
    driverOI.getShooterRollerButton().and(demoPreset).and(demoFeed).whileActiveContinuous(runShooterPreset);

    operatorOI.getShooterFlywheelStopButton().or(demoFeed).cancelWhenActive(runShooterManualHood)
        .cancelWhenActive(runShooterAutoHood).cancelWhenActive(runShooterPreset);
    operatorOI.updateLED(OILED.SHOOTER_STOP, OILEDState.ON);

    operatorOI.getHoodWallButton().and(manualHood).whenActive(() -> shooterHood.setTargetPosition(HoodPosition.WALL));
    operatorOI.getHoodFrontLineButton().and(manualHood)
        .whenActive(() -> shooterHood.setTargetPosition(HoodPosition.FRONT_LINE));
    operatorOI.getHoodBackLineButton().and(manualHood)
        .whenActive(() -> shooterHood.setTargetPosition(HoodPosition.BACK_LINE));
    operatorOI.getHoodTrenchButton().and(manualHood)
        .whenActive(() -> shooterHood.setTargetPosition(HoodPosition.TRENCH));

    operatorOI.getClimbEnableSwitch().whenActive(climber::deploy, climber);
    operatorOI.getClimbEnableSwitch().whenInactive(climber::reset, climber);
    operatorOI.getClimbEnableSwitch()
        .whileActiveContinuous(new RunClimber(climber, operatorOI::getClimbStickY, operatorOI::getClimbStickX));

    operatorOI.getGalacticSearchButton().and(new Trigger(() -> DriverStation.getInstance().isDisabled()))
        .whenActive(new InstantCommand() {
          @Override
          public void initialize() {
            galacticSearchCommand.updateVision();
          }

          @Override
          public boolean runsWhenDisabled() {
            return true;
          }
        });

    // ----- POWER PORT AUTO FOR AT HOME CHALLENGES -----
    // operatorOI.getPowerPortAutoButton().whileActiveOnce(new
    // PowerPortChallengeAuto(driveSubsystem, odometry, intake, hopper,
    // shooterRoller, shooterFlyWheel, shooterHood, limelight));

    operatorOI.getClimbEnableSwitch()
        .whenActive(new SetLEDOverride(OILED.CLIMB_ENABLE, OILEDState.ON, operatorOI::updateLED));
    operatorOI.getClimbEnableSwitch()
        .whenInactive(new SetLEDOverride(OILED.CLIMB_ENABLE, OILEDState.OFF, operatorOI::updateLED));
    operatorOI.updateLED(OILED.CLIMB_ENABLE, operatorOI.getClimbEnableSwitch().get() ? OILEDState.ON : OILEDState.OFF);

    operatorOI.getManualHoodSwitch()
        .whenActive(new SetLEDOverride(OILED.MANUAL_HOOD, OILEDState.ON, operatorOI::updateLED));
    operatorOI.getManualHoodSwitch()
        .whenInactive(new SetLEDOverride(OILED.MANUAL_HOOD, OILEDState.OFF, operatorOI::updateLED));
    operatorOI.updateLED(OILED.MANUAL_HOOD, operatorOI.getManualHoodSwitch().get() ? OILEDState.ON : OILEDState.OFF);

    driverOverrideOI.getLimelightLEDDisableSwitch()
        .whenActive(new SetLEDOverride(OILED.LIMELIGHT_DISABLE, OILEDState.ON, operatorOI::updateLED));
    driverOverrideOI.getLimelightLEDDisableSwitch()
        .whenInactive(new SetLEDOverride(OILED.LIMELIGHT_DISABLE, OILEDState.OFF, operatorOI::updateLED));
    operatorOI.updateLED(OILED.LIMELIGHT_DISABLE,
        driverOverrideOI.getLimelightLEDDisableSwitch().get() ? OILEDState.ON : OILEDState.OFF);

    driverOI.getAutoAimButton().whileActiveOnce(new PointAtTargetWithOdometry(driveSubsystem, odometry, limelight));
    RunMotionProfile autoDriveCommand = new RunMotionProfile(driveSubsystem, odometry, List.of(),
        new Pose2d(Constants.visionTargetHorizDist, Constants.fieldLength - Constants.initiationLine,
            Rotation2d.fromDegrees(0)),
        0, false, false);
    driverOI.getAutoDriveButton().whileActiveContinuous(autoDriveCommand);
    driverOI.getAutoDriveButton().whenInactive(autoDriveCommand::cancel);
  }

  public void updateOITimer() {
    if (operatorOI != null) {
      operatorOI.updateTimer();
    }
  }

  private void setupJoystickModeChooser() {
    joystickModeChooser = new SendableChooser<JoystickMode>();
    joystickModeChooser.addOption("Tank", JoystickMode.Tank);
    joystickModeChooser.addOption("Split Arcade", JoystickMode.SplitArcade);
    joystickModeChooser.setDefaultOption("Hybrid Curvature", JoystickMode.HybridCurvature);
    joystickModeChooser.addOption("Manual Curvature", JoystickMode.ManualCurvature);
    joystickModeChooser.addOption("Split Arcade (Southpaw)", JoystickMode.SplitArcadeSouthpaw);
    joystickModeChooser.addOption("Hybrid Curvature (Southpaw)", JoystickMode.HybridCurvatureSouthpaw);
    joystickModeChooser.addOption("Manual Curvature (Southpaw)", JoystickMode.ManualCurvatureSouthpaw);
    if (driverOI.hasDriveTriggers()) {
      joystickModeChooser.addOption("Trigger", JoystickMode.Trigger);
      joystickModeChooser.addOption("Trigger (Hybrid Curvature)", JoystickMode.TriggerHybridCurvature);
      joystickModeChooser.addOption("Trigger (Manual Curvature)", JoystickMode.TriggerManualCurvature);
    }
    SmartDashboard.putData("Joystick Mode", joystickModeChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void coastIfNotMoving() {
    if (!changedToCoast && Math.abs(driveSubsystem.getVelocityLeft()) <= 1
        && Math.abs(driveSubsystem.getVelocityRight()) <= 1) {
      driveSubsystem.enableBrakeMode(false);
      changedToCoast = true;
    }
  }

  public void brakeDuringNeutral() {
    driveSubsystem.enableBrakeMode(true);
    changedToCoast = false;
  }

  public void setInitialPosition() {
    ahrs.zeroYaw();
    odometry.setPosition(initialAutoPosition);
  }

  public enum FeedMode {
    NORMAL, DEMO, ACCURACY
  }
}
