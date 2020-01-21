/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithJoysticks.JoystickMode;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LimelightTest;
import frc.robot.oi.DummyOI;
import frc.robot.oi.OI;
import frc.robot.oi.OIConsole;
import frc.robot.oi.OIHandheld;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.drive.CTREDriveTrain;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final CameraSystem cameraSubsystem = new CameraSystem();
  private final LimelightInterface limelight = new LimelightInterface();
  private DriveTrainBase driveSubsystem;

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SendableChooser<JoystickMode> joystickModeChooser = new SendableChooser<JoystickMode>();

  private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);

  private OI oi = new DummyOI();
  private String lastJoystickName;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The subsystems can't be recreated when OI changes so provide them with a
    // BooleanSupplier to access the current value from whatever OI is current
    BooleanSupplier openLoopSwitchAccess = () -> oi.getOpenLoopSwitch().get();
    BooleanSupplier driveDisableSwitchAccess = () -> oi.getDriveDisableSwitch().get();
    BooleanSupplier shiftLockSwitchAccess = () -> oi.getShiftLockSwitch().get();
    switch (Constants.getRobot()) {
    case ROBOT_2020:
    case ROBOT_2020_DRIVE:
      // driveSubsystem = new SparkMAXDriveTrain(driveDisableSwitchAccess,
      // openLoopSwitchAccess);
      break;
    case ROBOT_2019:
    case ORIGINAL_ROBOT_2018:
    case REBOT:
    case NOTBOT:
      driveSubsystem = new CTREDriveTrain(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
      break;
    }

    joystickModeChooser.addOption("Tank", JoystickMode.Tank);
    if (oi.hasDriveTriggers()) {
      joystickModeChooser.addOption("Trigger", JoystickMode.Trigger);
    }
    joystickModeChooser.setDefaultOption("Split Arcade", JoystickMode.SplitArcade);
    joystickModeChooser.addOption("Split Arcade (right drive)", JoystickMode.SplitArcadeRightDrive);
  }

  public void updateOIType() {
    String joystickName = new Joystick(0).getName();
    if (joystickName != lastJoystickName) {
      // Button mapping must be cleared before instantiating new OI because the new OI
      // might need to map buttons internally
      CommandScheduler.getInstance().clearButtons();
      switch (joystickName) {
      case "Logitech Attack 3":
        oi = new OIConsole();
        break;
      case "XBox 360 Controller": // TODO Check this name
      case "Logitech F310 Gamepad":
        oi = new OIHandheld();
        break;
      default:
        oi = new DummyOI();
        break;
      }
      lastJoystickName = joystickName;
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
    DriveWithJoysticks driveCommand = new DriveWithJoysticks(oi::getLeftDriveX, oi::getLeftDriveY,
        oi::getLeftDriveTrigger, oi::getRightDriveX, oi::getRightDriveY, oi::getRightDriveTrigger, oi::getDeadband,
        oi.hasDriveTriggers(), oi::getSniperMode, oi::getSniperLevel, oi::getSniperHighLevel, oi::getSniperLowLevel,
        oi::getSniperLow, oi::getSniperHigh, oi.hasDualSniperMode(), joystickModeChooser, driveSubsystem);
    driveSubsystem.setDefaultCommand(driveCommand);
    // oi.getJoysticksForwardButton().whenActive(new InstantCommand(() ->
    // driveCommand.setReversed(false)));
    // oi.getJoysticksReverseButton().whenActive(new InstantCommand(() ->
    // driveCommand.setReversed(true)));
    // The DriveTrain will enforce the switches but this makes sure they are applied
    // immediately
    // neutralOutput is safer than stop since it prevents the motors from running
    oi.getDriveDisableSwitch().whenActive(new InstantCommand(driveSubsystem::neutralOutput, driveSubsystem));
    // This prevents the drive train from running in the old mode and since behavior
    // changes anyway stopping if the current command isn't calling drive is
    // reasonable. A brief neutralOutput while driving won't cause a noticable
    // change anyway
    oi.getOpenLoopSwitch().whenActive(new InstantCommand(driveSubsystem::neutralOutput));
    oi.getOpenLoopSwitch().whenInactive(new InstantCommand(driveSubsystem::neutralOutput));

    oi.getHighGearButton()
        .whenActive(new InstantCommand(() -> driveSubsystem.switchGear(DriveGear.HIGH), driveSubsystem));
    oi.getLowGearButton()
        .whenActive(new InstantCommand(() -> driveSubsystem.switchGear(DriveGear.LOW), driveSubsystem));
    oi.getToggleGearButton().whenActive(
        new InstantCommand(() -> driveSubsystem.switchGear(driveSubsystem.getCurrentGear().invert()), driveSubsystem));

    // Since useFrontCamera/useSecondCamera don't need arguments they can be passed
    // directly to InstantCommand
    oi.getFrontCameraButton().whenActive(new InstantCommand(cameraSubsystem::useFrontCamera, cameraSubsystem));
    oi.getSecondCameraButton().whenActive(new InstantCommand(cameraSubsystem::useSecondCamera, cameraSubsystem));

    oi.getVisionTestButton().whenActive(new LimelightTest(limelight, ahrs));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
