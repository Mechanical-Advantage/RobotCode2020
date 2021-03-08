// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.util.PolynomialRegression;

public class RunShooterAtDistance extends CommandBase {
  private static final double maxWallDistance = 110; // inches to center of robot
  private static final double minLineDistance = 110; // inches to center of robot
  private static final double maxLineDistance = 210; // inches to center of robot
  private static final double minTrenchDistance = 190; // inches to center of robot
  private static final double maxFlywheelSpeed = 6500; // RPM

  private static final PolynomialRegression wallRegression = new PolynomialRegression(
      new double[] { 31, 42, 56, 75, 90 }, new double[] { 6000, 4700, 3400, 3200, 3300 }, 2);
  private static final PolynomialRegression lineRegression = new PolynomialRegression(
      new double[] { 123, 145, 169, 195, 226 }, new double[] { 5700, 5500, 5100, 5100, 5100 }, 2);
  private static final PolynomialRegression trenchRegression = new PolynomialRegression(
      new double[] { 172, 181, 203, 226, 244, 263, 286 }, new double[] { 6300, 6100, 6000, 6000, 5900, 6050, 6050 }, 2);

  private final ShooterFlyWheel shooterFlyWheel;
  private final ShooterHood shooterHood;
  private final RobotOdometry odometry;
  private final Translation2d staticPosition;
  private final boolean autoHood;

  /**
   * Creates a new RunShooterAtDistance, which updates flywheel speed and hood
   * position once based on a known pose.
   */
  public RunShooterAtDistance(ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, Translation2d position,
      boolean autoHood) {
    this.shooterFlyWheel = shooterFlyWheel;
    this.shooterHood = shooterHood;
    this.odometry = null;
    this.staticPosition = position;
    this.autoHood = autoHood;
    addRequirements(shooterFlyWheel, shooterHood);
  }

  /**
   * Creates a new RunShooterAtDistance, which updates flywheel speed and hood
   * position continously based on odometry.
   */
  public RunShooterAtDistance(ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, RobotOdometry odometry,
      boolean autoHood) {
    this.shooterFlyWheel = shooterFlyWheel;
    this.shooterHood = shooterHood;
    this.odometry = odometry;
    this.staticPosition = null;
    this.autoHood = autoHood;
    addRequirements(shooterFlyWheel, shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    // NOTE: This distance calculation should be based on the distance to the inner
    // port
    double distance = position
        .getDistance(new Translation2d(Constants.fieldLength, Constants.visionTargetHorizDist * -1));
    if (Constants.flatTarget) {
      distance -= Constants.innerPortDepth;
    }

    // Update hood position
    if (autoHood) {
      double wallLineTransition, lineTrenchTransition;
      switch (shooterHood.getTargetPosition()) {
        case WALL:
          wallLineTransition = maxWallDistance;
          lineTrenchTransition = maxLineDistance;
          break;
        case LINE:
          wallLineTransition = minLineDistance;
          lineTrenchTransition = maxLineDistance;
          break;
        case TRENCH:
          wallLineTransition = minLineDistance;
          lineTrenchTransition = minTrenchDistance;
          break;
        case UNKNOWN:
        default:
          wallLineTransition = (maxWallDistance + minLineDistance) / 2;
          lineTrenchTransition = (maxLineDistance + minTrenchDistance) / 2;
          break;
      }
      if (distance < wallLineTransition) {
        shooterHood.setTargetPosition(HoodPosition.WALL);
      } else if (distance < lineTrenchTransition) {
        shooterHood.setTargetPosition(HoodPosition.LINE);
      } else {
        shooterHood.setTargetPosition(HoodPosition.TRENCH);
      }
    }

    // Update flywheel speed
    double predictedSpeed;
    switch (shooterHood.getTargetPosition()) {
      case WALL:
        predictedSpeed = wallRegression.predict(distance);
        break;
      case LINE:
        predictedSpeed = lineRegression.predict(distance);
        break;
      case TRENCH:
        predictedSpeed = trenchRegression.predict(distance);
        break;
      default:
        predictedSpeed = 0;
        break;
    }
    shooterFlyWheel.setShooterRPM(predictedSpeed > maxFlywheelSpeed ? maxFlywheelSpeed : predictedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterFlyWheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
