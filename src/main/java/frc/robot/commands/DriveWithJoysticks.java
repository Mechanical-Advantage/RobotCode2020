/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrainBase;

public class DriveWithJoysticks extends CommandBase {
  /**
   * Creates a new DriveWithJoysticks.
   */

  private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or
  // of current gear
  private static final double rightStickScale = 0.5; // Factor of right stick when added

  private DoubleSupplier oiLeftDriveX;
  private DoubleSupplier oiLeftDriveY;
  private DoubleSupplier oiLeftDriveTrigger;
  private DoubleSupplier oiRightDriveX;
  private DoubleSupplier oiRightDriveY;
  private DoubleSupplier oiRightDriveTrigger;
  private DoubleSupplier oiGetDeadband;
  private boolean oiHasDriveTriggers;
  private BooleanSupplier oiSniperMode;
  private DoubleSupplier oiSniperLevel;
  private DoubleSupplier oiSniperHighLevel;
  private DoubleSupplier oiSniperLowLevel;
  private BooleanSupplier oiSniperLow;
  private BooleanSupplier oiSniperHigh;
  private boolean oiHasDualSniperMode;
  private DriveTrainBase driveSubsystem;
  private SendableChooser<JoystickMode> joystickChooser;

  private boolean joysticksReversed = false;

  public DriveWithJoysticks(DoubleSupplier leftDriveX, DoubleSupplier leftDriveY, DoubleSupplier leftDriveTrigger,
      DoubleSupplier rightDriveX, DoubleSupplier rightDriveY, DoubleSupplier rightDriveTrigger,
      DoubleSupplier getDeadband, boolean hasDriveTriggers, BooleanSupplier sniperMode, DoubleSupplier sniperLevel,
      DoubleSupplier sniperHighLevel, DoubleSupplier sniperLowLevel, BooleanSupplier sniperLow,
      BooleanSupplier sniperHigh, boolean hasDualSniperMode, SendableChooser<JoystickMode> joystickModeChooser,
      DriveTrainBase driveSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super();
    addRequirements(driveSubsystem);
    oiLeftDriveX = leftDriveX;
    oiLeftDriveY = leftDriveY;
    oiLeftDriveTrigger = leftDriveTrigger;
    oiRightDriveX = rightDriveX;
    oiRightDriveY = rightDriveY;
    oiRightDriveTrigger = rightDriveTrigger;
    oiGetDeadband = getDeadband;
    oiHasDriveTriggers = hasDriveTriggers;
    oiSniperMode = sniperMode;
    oiSniperLevel = sniperLevel;
    oiSniperHighLevel = sniperHighLevel;
    oiSniperLowLevel = sniperLowLevel;
    oiSniperLow = sniperLow;
    oiSniperHigh = sniperHigh;
    oiHasDualSniperMode = hasDualSniperMode;
    joystickChooser = joystickModeChooser;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Driver Control", true);
  }

  private double processJoystickAxis(double joystickAxis) {
    // cube to improve low speed control, multiply by -1 because negative joystick
    // means forward, 0 if within deadband
    return Math.abs(joystickAxis) > oiGetDeadband.getAsDouble() ? joystickAxis * Math.abs(joystickAxis) * -1 : 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickLeft = 0, joystickRight = 0;
    double baseDrive;
    double totalTurn;
    switch (joystickChooser.getSelected()) {
    // TODO check that joystickReversed values are correct.
    // TODO shorten joysticksReversed logic so not as repetative.
    case Tank:
      joystickRight = processJoystickAxis(oiLeftDriveY.getAsDouble() /* Robot.oi.getRightAxis() */);
      joystickLeft = processJoystickAxis(oiRightDriveY.getAsDouble() /* Robot.oi.getLeftAxis() */);
      if (joysticksReversed) {
        joystickRight *= -1;
        joystickLeft *= -1;
      }
      break;
    case SplitArcade:
      baseDrive = processJoystickAxis(oiLeftDriveY.getAsDouble() /* Robot.oi.getSingleDriveAxisLeft() */);
      joystickRight = baseDrive
          + processJoystickAxis(oiRightDriveX.getAsDouble() /* Robot.oi.getRightHorizDriveAxis() */);
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive
          - processJoystickAxis(oiRightDriveX.getAsDouble() /* Robot.oi.getRightHorizDriveAxis() */);
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;

      if (joysticksReversed) {
        baseDrive *= -1;
        joystickRight = baseDrive - processJoystickAxis(oiRightDriveX.getAsDouble());
        joystickRight = joystickRight > 1 ? 1 : joystickRight;
        joystickLeft = baseDrive + processJoystickAxis(oiRightDriveX.getAsDouble());
        joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      }
      break;
    case SplitArcadeRightDrive:
      baseDrive = processJoystickAxis(oiRightDriveY.getAsDouble() /* Robot.oi.getSingleDriveAxisRight() */);
      joystickRight = baseDrive
          + processJoystickAxis(oiLeftDriveX.getAsDouble() /* Robot.oi.getLeftHorizDriveAxis() */);
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(oiLeftDriveX.getAsDouble() /* Robot.oi.getLeftHorizDriveAxis() */);
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;

      if (joysticksReversed) {
        baseDrive *= -1;
        joystickRight = baseDrive - processJoystickAxis(oiLeftDriveX.getAsDouble());
        joystickRight = joystickRight > 1 ? 1 : joystickRight;
        joystickLeft = baseDrive + processJoystickAxis(oiLeftDriveX.getAsDouble());
        joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      }
      break;
    case Trigger:
      baseDrive = (oiLeftDriveTrigger.getAsDouble()
          /* Robot.oi.getLeftTrigger() */ - oiRightDriveTrigger.getAsDouble() /* Robot.oi.getRightTrigger() */ ) * -1;
      totalTurn = oiLeftDriveX.getAsDouble() /* Robot.oi.getLeftHorizDriveAxis() */
          + (oiRightDriveX.getAsDouble() /* Robot.oi.getRightHorizDriveAxis() */ * rightStickScale);
      joystickRight = baseDrive + processJoystickAxis(totalTurn);
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(totalTurn);
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;

      if (joysticksReversed) {
        baseDrive = (oiRightDriveTrigger.getAsDouble() - oiLeftDriveTrigger.getAsDouble()) * -1;
        totalTurn = oiLeftDriveX.getAsDouble() + (oiRightDriveX.getAsDouble() * rightStickScale);

        joystickRight = baseDrive - processJoystickAxis(totalTurn);
        joystickRight = joystickRight > 1 ? 1 : joystickRight;

        joystickLeft = baseDrive + processJoystickAxis(totalTurn);
        joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      }
      break;
    }
    if (oiSniperMode.getAsBoolean()) {
      getMultiplierForSniperMode();
      double multiplierLevel = getMultiplierForSniperMode();
      joystickLeft *= multiplierLevel;
      joystickRight *= multiplierLevel;
    }

    driveSubsystem.drive(joystickLeft, joystickRight, alwaysUseHighMaxVel);
  }

  public double getMultiplierForSniperMode() {
    double level;
    if (oiHasDualSniperMode) {
      if (oiSniperHigh.getAsBoolean()) {
        level = oiSniperLowLevel.getAsDouble();
      } else {
        level = oiSniperHighLevel.getAsDouble();
      }
    } else {
      level = oiSniperLevel.getAsDouble();
    }
    return level;
  }

  public void setReversed(boolean reverse) {
    joysticksReversed = reverse;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    SmartDashboard.putBoolean("Driver Control", false);
  }

  public static enum JoystickMode {
    Tank, SplitArcade, SplitArcadeRightDrive, Trigger;
  }
}
