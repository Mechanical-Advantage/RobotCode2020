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
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedUnstick;
import frc.robot.commands.IdleLimelightControl;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.commands.LimelightOdometry;
import frc.robot.commands.LimelightTest;
import frc.robot.commands.PointAtTarget;
import frc.robot.commands.PointAtTargetAndShoot;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntakeBackwards;
import frc.robot.commands.RunIntakeForwards;
import frc.robot.commands.RunMotionProfile;
import frc.robot.commands.RunShooterFlyWheel;
import frc.robot.commands.RunShooterRoller;
import frc.robot.commands.SetLEDOverride;
import frc.robot.commands.SetShooterHoodMiddleTop;
import frc.robot.commands.SetShooterHoodBottom;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.VelocityPIDTuner;
import frc.robot.oi.DummyOI;
import frc.robot.oi.IDriverOI;
import frc.robot.oi.IDriverOverrideOI;
import frc.robot.oi.IOperatorOI;
import frc.robot.oi.OIArduinoConsole;
import frc.robot.oi.OIDualJoysticks;
import frc.robot.oi.OIHandheld;
import frc.robot.oi.OIHandheldAllInOne;
import frc.robot.oi.OIeStopConsole;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.CTREDriveTrain;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;
import frc.robot.util.PressureSensor;
import frc.robot.subsystems.drive.SparkMAXDriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double navXWaitTime = 5; // Maximum number of seconds to wait for the navX to initialize
  private static final Pose2d initialAutoPosition = new Pose2d(Constants.fieldLength - Constants.initiationLine, 0,
      Rotation2d.fromDegrees(0));

  private IDriverOI driverOI;
  private IDriverOverrideOI driverOverrideOI;
  private IOperatorOI operatorOI;
  private String[] lastJoystickNames;
  private boolean changedToCoast;
  private int lastMatchTime;

  // The robot's subsystems and commands are defined here...
  private final CameraSystem cameraSubsystem = new CameraSystem();
  private final LimelightInterface limelight = new LimelightInterface();
  private DriveTrainBase driveSubsystem;
  private final ShooterFlyWheel shooterFlyWheel = new ShooterFlyWheel((led, state) -> operatorOI.updateLED(led, state),
      (double rpm) -> operatorOI.setFlyWheelSpeed(rpm));
  private final ShooterRoller shooterRoller = new ShooterRoller();
  private final ShooterHood shooterHood = new ShooterHood((led, state) -> operatorOI.updateLED(led, state),
      (position) -> operatorOI.setHoodPosition(position));
  private final Intake intake = new Intake((led, state) -> operatorOI.updateLED(led, state));
  private final Hopper hopper = new Hopper();
  private final Climber climber = new Climber();
  private RobotOdometry odometry;
  private final PressureSensor pressureSensor = new PressureSensor(0, (pressure) -> operatorOI.setPressure(pressure));

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private SendableChooser<JoystickMode> joystickModeChooser;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private LimelightOdometry limelightOdometry;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Use a dummy OI initially
    DummyOI dummyOI = new DummyOI();
    driverOI = dummyOI;
    driverOverrideOI = dummyOI;
    operatorOI = dummyOI;
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
      DriverStation.reportError("Timeout while waiting for NavX init", false);
    }
    odometry = new RobotOdometry(driveSubsystem, ahrs);
    limelightOdometry = new LimelightOdometry(limelight, odometry);
    odometry.setDefaultCommand(limelightOdometry);
    limelight.setDefaultCommand(
        new IdleLimelightControl(limelight, () -> driverOverrideOI.getLimelightLEDDisableSwitch().get()));

    setupJoystickModeChooser();

    autoChooser.setDefaultOption("Do Nothing", null);
    if (Constants.tuningMode) {
      autoChooser.addOption("Turn 90 degrees", new TurnToAngle(driveSubsystem, ahrs, 90));
      autoChooser.addOption("Turn 15 degrees", new TurnToAngle(driveSubsystem, ahrs, 15));
      autoChooser.addOption("Drive 5 feet", new DriveDistanceOnHeading(driveSubsystem, ahrs, 60));
      autoChooser.addOption("Drive velocity", new VelocityPIDTuner(driveSubsystem));
      autoChooser.addOption("Drive 5 feet (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(0, 60, new Rotation2d(0)), 0, false, true));
      autoChooser.addOption("Drive to 5 feet absolute (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(0, 60, new Rotation2d(0)), 0, false, false));
      autoChooser.addOption("Drive 5 foot arc (MP)", new RunMotionProfile(driveSubsystem, odometry, List.of(),
          new Pose2d(180, 60, Rotation2d.fromDegrees(90)), 0, false, true));
    }
    autoChooser.addOption("Aim and fire loaded balls",
        new PointAtTargetAndShoot(driveSubsystem, limelight, ahrs, hopper, shooterRoller, shooterFlyWheel));
    SmartDashboard.putData("Auto Mode", autoChooser);
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

      // The first name that is seen will be used and any other
      // controller names will be ignored (will only complete a pair)
      int firstController = 0; // Used to store first ID for controller pairs
      String firstControllerName = null;
      String joystickName;
      boolean operatorOIFound = false;
      boolean driverOIFound = false;

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
              operatorOIFound = true;
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
              operatorOIFound = true;
              System.out.println("Operator: eStop console");
            }
            break;
        }
      }

      // Look for driver controller
      firstControllerName = null;
      for (joystickNum = 0; joystickNum < 6; joystickNum++) {
        joystickName = joystickNames[joystickNum];
        switch (joystickNames[joystickNum]) {
          case "Controller (XBOX 360 For Windows)":
          case "Controller (Gamepad F310)":
          case "XBOX 360 For Windows (Controller)":
          case "Gamepad F310 (Controller)":
            if (firstControllerName == null) {
              firstControllerName = joystickName;
              if (operatorOIFound) {
                driverOI = new OIHandheld(joystickNum);
                System.out.println("Driver: XBox/F310 controller");
              } else {
                OIHandheldAllInOne handheldAllInOne = new OIHandheldAllInOne(joystickNum);
                driverOI = handheldAllInOne;
                operatorOI = handheldAllInOne;
                driverOverrideOI = handheldAllInOne;
                operatorOIFound = true;
                System.out.println("Driver/operator: XBox/F310 controller");
              }
              driverOIFound = true;
            }
            break;
          case "Logitech Attack 3":
            if (firstControllerName == null) {
              firstController = joystickNum;
              firstControllerName = joystickName;
            } else if (firstControllerName.equals(joystickName)) {
              driverOI = new OIDualJoysticks(firstController, joystickNum);
              driverOIFound = true;
              System.out.println("Driver: Dual Attack 3");
            }
            break;
        }
      }

      DummyOI dummyOI = null;
      if (!operatorOIFound) {
        DriverStation.reportWarning("No operator controller found", false);
        dummyOI = new DummyOI();
        operatorOI = dummyOI;
        // At the moment all the operator OIs also implement IDriverOverrideOI
        driverOverrideOI = dummyOI;
      }
      if (!driverOIFound) {
        DriverStation.reportWarning("No driver controller found", false);
        if (dummyOI == null) {
          dummyOI = new DummyOI();
        }
        driverOI = dummyOI;
      }

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
        driverOI::getRightDriveTrigger, driverOI::getDeadband, driverOI::getSniperMode, driverOI::getSniperLevel,
        driverOI::getSniperHighLevel, driverOI::getSniperLowLevel, driverOI::getSniperLow, driverOI::getSniperHigh,
        driverOI.hasDualSniperMode(), joystickModeChooser, driveSubsystem);
    driveSubsystem.setDefaultCommand(driveCommand);
    driverOI.getJoysticksForwardButton().whenActive(() -> driveCommand.setReversed(false));
    driverOI.getJoysticksReverseButton().whenActive(() -> driveCommand.setReversed(true));

    driverOverrideOI.getOpenLoopSwitch()
        .whenActive(new SetLEDOverride(OILED.OPEN_LOOP, OILEDState.ON, operatorOI::updateLED));
    driverOverrideOI.getOpenLoopSwitch()
        .whenInactive(new SetLEDOverride(OILED.OPEN_LOOP, OILEDState.OFF, operatorOI::updateLED));

    driverOverrideOI.getDriveDisableSwitch()
        .whenActive(new SetLEDOverride(OILED.DRIVE_DISABLE, OILEDState.ON, operatorOI::updateLED));
    driverOverrideOI.getDriveDisableSwitch()
        .whenInactive(new SetLEDOverride(OILED.DRIVE_DISABLE, OILEDState.OFF, operatorOI::updateLED));
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

    operatorOI.getShooterRollerButton()
        .whileActiveContinuous(new RunShooterRoller(shooterRoller).alongWith(new RunHopper(hopper)));
    operatorOI.getShooterUnstickButton()
        .whileActiveContinuous(new FeedUnstick(shooterRoller, hopper, operatorOI::updateLED));

    operatorOI.getIntakeExtendButton().whenActive(() -> intake.extend());
    operatorOI.getIntakeRetractButton().whenActive(() -> intake.retract());
    operatorOI.updateLED(OILED.INTAKE_RETRACT, OILEDState.ON);

    RunIntakeForwards runIntakeForwards = new RunIntakeForwards(intake);
    RunIntakeBackwards runIntakeBackwards = new RunIntakeBackwards(intake);
    operatorOI.getRunIntakeForwardsButton().whileActiveContinuous(runIntakeForwards);
    operatorOI.getRunIntakeBackwardsButton().whileActiveContinuous(runIntakeBackwards);

    RunShooterFlyWheel runShooter = new RunShooterFlyWheel(shooterFlyWheel);
    operatorOI.getShooterFlywheelRunButton().whenActive(runShooter);
    operatorOI.getShooterFlywheelStopButton().cancelWhenActive(runShooter);
    operatorOI.updateLED(OILED.SHOOTER_STOP, OILEDState.ON);

    operatorOI.getHoodWallButton().and(operatorOI.getManualHoodSwitch())
        .whenActive(new SetShooterHoodBottom(shooterHood));
    operatorOI.getHoodLineButton().and(operatorOI.getManualHoodSwitch())
        .whenActive(new SetShooterHoodMiddleTop(shooterHood, pressureSensor, false));
    operatorOI.getHoodTrenchButton().and(operatorOI.getManualHoodSwitch())
        .whenActive(new SetShooterHoodMiddleTop(shooterHood, pressureSensor, true));

    operatorOI.getClimbEnableSwitch().whenActive(climber::deploy, climber);
    operatorOI.getClimbEnableSwitch().whenInactive(climber::reset, climber);
    operatorOI.getClimbEnableSwitch().whileActiveContinuous(new RunClimber(climber, operatorOI::getClimbStickY));

    operatorOI.getClimbEnableSwitch()
        .whenActive(new SetLEDOverride(OILED.CLIMB_ENABLE, OILEDState.ON, operatorOI::updateLED));
    operatorOI.getClimbEnableSwitch()
        .whenInactive(new SetLEDOverride(OILED.CLIMB_ENABLE, OILEDState.OFF, operatorOI::updateLED));

    operatorOI.getManualHoodSwitch()
        .whenActive(new SetLEDOverride(OILED.MANUAL_HOOD, OILEDState.ON, operatorOI::updateLED));
    operatorOI.getManualHoodSwitch()
        .whenInactive(new SetLEDOverride(OILED.MANUAL_HOOD, OILEDState.OFF, operatorOI::updateLED));

    PointAtTarget autoAimCommand = new PointAtTarget(driveSubsystem, limelight, ahrs);
    driverOI.getAutoAimButton().whenActive(autoAimCommand);
    driverOI.getAutoAimButton().whenInactive(autoAimCommand::cancel);
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
    if (driverOI.hasDriveTriggers()) {
      joystickModeChooser.addOption("Trigger", JoystickMode.Trigger);
    }
    joystickModeChooser.setDefaultOption("Split Arcade", JoystickMode.SplitArcade);
    joystickModeChooser.addOption("Split Arcade (right drive)", JoystickMode.SplitArcadeRightDrive);
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
    odometry.setPosition(initialAutoPosition);
  }

  public void enableLimelightXCorrection(boolean enable) {
    limelightOdometry.enableXCorrection(enable);
  }
}
