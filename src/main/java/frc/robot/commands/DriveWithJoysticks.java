/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoysticks extends CommandBase {
  /**
   * Creates a new DriveWithJoysticks.
   */

  private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or
  // of current gear
  private static final double rightStickScale = 0.5; // Factor of right stick when added

  public DriveWithJoysticks(// add types
  leftDriveX, leftDriveY, leftDriveTrigger, rightDriveX, rightDriveY, rightDriveTrigger, 
  hasDriveTriggers, sniperMode, sniperLevel, sniperLow, sniperHigh, hasDualSniperMode) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super("DriveWithJoystick");
    requires(Robot.driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Driver Control", true);
  }

  private double processJoystickAxis(double joystickAxis) {
    // cube to improve low speed control, multiply by -1 because negative joystick
    // means forward, 0 if within deadband
    return Math.abs(joystickAxis) > Robot.oi.getDeadband() ? joystickAxis * Math.abs(joystickAxis) * -1 : 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickLeft = 0, joystickRight = 0;
    double baseDrive;
    double totalTurn;
    switch (Robot.joystickModeChooser.getSelected()) {
    case Tank:
      joystickRight = processJoystickAxis(Robot.oi.getRightAxis());
      joystickLeft = processJoystickAxis(Robot.oi.getLeftAxis());
      break;
    case SplitArcade:
      baseDrive = processJoystickAxis(Robot.oi.getSingleDriveAxisLeft());
      joystickRight = baseDrive + processJoystickAxis(Robot.oi.getRightHorizDriveAxis());
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(Robot.oi.getRightHorizDriveAxis());
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    case SplitArcadeRightDrive:
      baseDrive = processJoystickAxis(Robot.oi.getSingleDriveAxisRight());
      joystickRight = baseDrive + processJoystickAxis(Robot.oi.getLeftHorizDriveAxis());
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(Robot.oi.getLeftHorizDriveAxis());
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    case Trigger:
      baseDrive = (Robot.oi.getLeftTrigger() - Robot.oi.getRightTrigger()) * -1;
      totalTurn = Robot.oi.getLeftHorizDriveAxis() + (Robot.oi.getRightHorizDriveAxis() * rightStickScale);
      joystickRight = baseDrive + processJoystickAxis(totalTurn);
      joystickRight = joystickRight > 1 ? 1 : joystickRight;
      joystickLeft = baseDrive - processJoystickAxis(totalTurn);
      joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
      break;
    case Sniper:

      break;
    }
    Robot.driveSubsystem.drive(joystickLeft, joystickRight, alwaysUseHighMaxVel);

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
    Tank, SplitArcade, SplitArcadeRightDrive, Trigger, Sniper;
  }
}
