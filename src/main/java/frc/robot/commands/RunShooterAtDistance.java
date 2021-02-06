// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.PressureSensor;

public class RunShooterAtDistance extends CommandBase {
  private static final double maxWallDistance = 115; // inches to center of robot
  private static final double maxLineDistance = 220; // inches to center of robot
  private static final double maxFlywheelSpeed = 6500; // RPM

  private static final PolynomialRegression wallRegression = new PolynomialRegression(
      new double[] { 31, 42, 56, 75, 90 }, new double[] { 6000, 4700, 3400, 3200, 3300 }, 2);
  private static final PolynomialRegression lineRegression = new PolynomialRegression(
      new double[] { 123, 145, 169, 195, 226 }, new double[] { 5700, 5500, 5100, 5100, 5100 }, 2);
  private static final PolynomialRegression trenchRegression = new PolynomialRegression(
      new double[] { 172, 181, 203, 226, 244, 263, 286 }, new double[] { 6300, 6100, 6000, 6000, 5900, 6050, 6050 }, 2);

  private final ShooterFlyWheel shooterFlyWheel;
  private final RobotOdometry odometry;
  private Translation2d staticPosition;

  private final Command wallCommand;
  private final Command lineCommand;
  private final Command trenchCommand;

  private HoodPosition lastCommandedPosition;

  /**
   * Creates a new RunShooterAtDistance, which updates flywheel speed and hood
   * position once based on a known pose.
   */
  public RunShooterAtDistance(ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, PressureSensor pressureSensor,
      UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD, Translation2d position) {
    this(shooterFlyWheel, shooterHood, null, pressureSensor, updateLED, setHoodLCD);
    this.staticPosition = position;
  }

  /**
   * Creates a new RunShooterAtDistance, which updates flywheel speed and hood
   * position continously based on odometry.
   */
  public RunShooterAtDistance(ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, RobotOdometry odometry,
      PressureSensor pressureSensor, UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    this.shooterFlyWheel = shooterFlyWheel;
    this.odometry = odometry;
    addRequirements(shooterFlyWheel);

    wallCommand = new SetShooterHoodBottom(shooterHood, updateLED, setHoodLCD);
    lineCommand = new SetShooterHoodMiddleTop(shooterHood, pressureSensor, false, updateLED, setHoodLCD);
    trenchCommand = new SetShooterHoodMiddleTop(shooterHood, pressureSensor, true, updateLED, setHoodLCD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Force the position to be commanded once
    lastCommandedPosition = null;

    if (staticPosition != null) {
      update(staticPosition);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (odometry != null) {
      update(odometry.getCurrentPose().getTranslation());
    }
  }

  private void update(Translation2d position) {
    double distance = position
        .getDistance(new Translation2d(Constants.fieldLength, Constants.visionTargetHorizDist * -1));

    // Determine zone
    HoodPosition bestPosition;
    PolynomialRegression regression;
    if (distance < maxWallDistance) {
      bestPosition = HoodPosition.WALL;
      regression = wallRegression;
    } else if (distance < maxLineDistance) {
      bestPosition = HoodPosition.LINE;
      regression = lineRegression;
    } else {
      bestPosition = HoodPosition.TRENCH;
      regression = trenchRegression;
    }

    // Update flywheel speed
    double predictedSpeed = regression.predict(distance);
    shooterFlyWheel.setShooterRPM(predictedSpeed > maxFlywheelSpeed ? maxFlywheelSpeed : predictedSpeed);

    // Update hood position
    if (bestPosition != lastCommandedPosition) {
      switch (bestPosition) {
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
      lastCommandedPosition = bestPosition;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterFlyWheel.stop();
    wallCommand.cancel();
    lineCommand.cancel();
    trenchCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
