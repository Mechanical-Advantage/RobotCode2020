/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.UtilFunctions;

/**
 * A command that does a simple spin in place at a fixed power until the
 * specified angle is reached. Expect overshoot.
 */
public class TurnToAngleFast extends CommandBase {

  private final DriveTrainBase driveTrain;
  private final AHRS ahrs;
  private final double commandedAngle;
  private final boolean absoluteAngle;
  private final double power;

  private double targetAngle;
  private Boolean spinLeft;

  /**
   * Creates a new TurnToAngleFast.
   * 
   * @param driveTrain    The drive train
   * @param ahrs          The NavX
   * @param angle         The target angle
   * @param absoluteAngle Whether the angle is absolute instead of relative to the
   *                      starting angle
   * @param power         The power level from 0-1 (never negative)
   */
  public TurnToAngleFast(DriveTrainBase driveTrain, AHRS ahrs, double angle, boolean absoluteAngle, double power) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    commandedAngle = angle;
    this.absoluteAngle = absoluteAngle;
    this.power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentAngle = ahrs.getAngle();
    if (!absoluteAngle) {
      currentAngle = 0;
    }
    double difference = UtilFunctions.boundHalfDegrees(commandedAngle - UtilFunctions.boundHalfDegrees(currentAngle));
    targetAngle = currentAngle + difference;
    if (difference > 0) {
      driveTrain.drive(power, power * -1);
      spinLeft = false;
    } else if (difference < 0) {
      driveTrain.drive(power * -1, power);
      spinLeft = true;
    } else {
      spinLeft = null;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (spinLeft == null) {
      return true;
    }
    double currentAngle = ahrs.getAngle();
    if (spinLeft) {
      return currentAngle <= targetAngle;
    } else {
      return currentAngle >= targetAngle;
    }
  }
}
