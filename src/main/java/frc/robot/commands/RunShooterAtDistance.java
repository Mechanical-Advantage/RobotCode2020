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
  private static final double maxWallDistance = 114; // inches to center of robot
  private static final double minLineDistance = 104; // inches to center of robot
  private static final double maxLineDistance = 199; // inches to center of robot
  private static final double minTrenchDistance = 194; // inches to center of robot
  private static final double maxFlywheelSpeed = 6500; // RPM

  private static final PolynomialRegression wallRegression = new PolynomialRegression(
      new double[] { 31 + Constants.innerPortDepth, 42 + Constants.innerPortDepth, 56 + Constants.innerPortDepth, 75 + Constants.innerPortDepth, 90 + Constants.innerPortDepth }, new double[] { 6000, 4700, 3400, 3200, 3300 }, 2);
  private static final PolynomialRegression lineRegression = new PolynomialRegression(
      new double[] { 75 + Constants.innerPortDepth, 81 + Constants.innerPortDepth, 87 + Constants.innerPortDepth, 93 + Constants.innerPortDepth, 99 + Constants.innerPortDepth, 105 + Constants.innerPortDepth, 111 + Constants.innerPortDepth, 117 + Constants.innerPortDepth, 123 + Constants.innerPortDepth, 129 + Constants.innerPortDepth, 132.5 + Constants.innerPortDepth, 135 + Constants.innerPortDepth, 138 + Constants.innerPortDepth, 144 + Constants.innerPortDepth, 147 + Constants.innerPortDepth },
      new double[] { 3625, 3600, 3600, 3625, 3650, 3675, 3700, 3750, 3750, 3800, 3800, 3825, 3875, 3925, 3925 }, 2);
  private static final PolynomialRegression trenchRegression = new PolynomialRegression(
      new double[] { 172 + Constants.innerPortDepth, 181 + Constants.innerPortDepth, 203 + Constants.innerPortDepth, 226 + Constants.innerPortDepth, 244 + Constants.innerPortDepth, 263 + Constants.innerPortDepth, 286 + Constants.innerPortDepth }, new double[] { 6300, 6100, 6000, 6000, 5900, 6050, 6050 }, 2);

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
        .getDistance(PointAtTargetWithOdometry.getTargetPosition(position));

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
