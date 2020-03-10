/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterHood;
import frc.robot.util.PressureSensor;

public class AutoShooterHood extends CommandBase {

  // When the robot's position between the ends of the field is within this
  // distance of the line go to line position
  private static final double lineTriggerDistance = 36;
  // When the robot is in the trench run or within this distance of it go to
  // trench position
  private static final double trenchRunMargin = 24;

  private final RobotOdometry odometry;

  private final Command wallCommand;
  private final Command lineCommand;
  private final Command trenchCommand;

  private HoodPosition lastCommandedPosition;

  /**
   * Creates a new AutoShooterHood.
   */
  public AutoShooterHood(ShooterHood hood, RobotOdometry odometry, PressureSensor pressureSensor,
      UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    // Nothing is required because the commands called here are what actually
    // intereact with the hood and this command must not have exclusive control of
    // the odometry
    this.odometry = odometry;

    wallCommand = new SetShooterHoodBottom(hood, updateLED, setHoodLCD);
    lineCommand = new SetShooterHoodMiddleTop(hood, pressureSensor, false, updateLED, setHoodLCD);
    trenchCommand = new SetShooterHoodMiddleTop(hood, pressureSensor, true, updateLED, setHoodLCD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Force the position to be commanded once
    lastCommandedPosition = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HoodPosition bestPosition = getBestPosition();
    if (bestPosition != lastCommandedPosition) {
      setPosition(bestPosition);
      lastCommandedPosition = bestPosition;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wallCommand.cancel();
    lineCommand.cancel();
    trenchCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private HoodPosition getBestPosition() {
    Translation2d currentTranslation = odometry.getCurrentPose().getTranslation();
    if (Math
        .abs(currentTranslation.getX() - (Constants.fieldLength - Constants.initiationLine)) <= lineTriggerDistance) {
      return HoodPosition.LINE;
    }
    // Don't take the absolute value of the y value because only the right side is
    // our trench, multiply by -1 because negative=right
    if (currentTranslation.getY() * -1 >= Constants.fieldWidth / 2 - Constants.trenchRunWidth - trenchRunMargin
        && Math.abs(currentTranslation.getX() - Constants.fieldLength / 2) <= Constants.trenchRunLength / 2
            + trenchRunMargin) {
      return HoodPosition.TRENCH;
    }
    // The wall is the default position because it is fully lowered
    return HoodPosition.WALL;
  }

  private void setPosition(HoodPosition position) {
    switch (position) {
      case WALL:
        wallCommand.schedule();
        break;
      case LINE:
        lineCommand.schedule();
        break;
      case TRENCH:
        trenchCommand.schedule();
        break;
    }
  }

  private static enum HoodPosition {
    WALL, LINE, TRENCH;
  }
}
