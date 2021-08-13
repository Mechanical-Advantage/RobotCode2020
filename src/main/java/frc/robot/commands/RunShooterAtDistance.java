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
  private static final double minFrontLineDistance = 104; // inches to center of robot
  private static final double maxFrontLineDistance = 169; // inches to center of robot
  private static final double minBackLineDistance = 165; // inches to center of robot
  private static final double maxBackLineDistance = 249; // inches to center of robot
  private static final double minTrenchDistance = 239; // inches to center of robot
  private static final double maxFlywheelSpeed = 6500; // RPM

  private static final PolynomialRegression wallInnerRegression = new PolynomialRegression(
      new double[] { 31 + Constants.innerPortDepth, 42 + Constants.innerPortDepth, 56 + Constants.innerPortDepth,
          75 + Constants.innerPortDepth, 90 + Constants.innerPortDepth },
      new double[] { 6000, 4700, 3400, 3200, 3300 }, 2);
  private static final PolynomialRegression wallOuterRegression = new PolynomialRegression(
      new double[] { 33.5, 36.5, 42.5, 48.5, 54.5, 60.5 }, new double[] { 6500, 4300, 4100, 3100, 3000, 3100 }, 2);
  private static final PolynomialRegression frontLineRegression = new PolynomialRegression(
      new double[] { 75 + Constants.innerPortDepth, 81 + Constants.innerPortDepth, 87 + Constants.innerPortDepth,
          93 + Constants.innerPortDepth, 99 + Constants.innerPortDepth, 105 + Constants.innerPortDepth,
          111 + Constants.innerPortDepth, 117 + Constants.innerPortDepth, 123 + Constants.innerPortDepth,
          129 + Constants.innerPortDepth, 132.5 + Constants.innerPortDepth, 135 + Constants.innerPortDepth,
          138 + Constants.innerPortDepth, 144 + Constants.innerPortDepth, 147 + Constants.innerPortDepth },
      new double[] { 3625, 3600, 3600, 3625, 3650, 3675, 3700, 3750, 3750, 3800, 3800, 3825, 3875, 3925, 3925 }, 2);
  private static final PolynomialRegression backLineRegression = new PolynomialRegression(
      new double[] { 123 + Constants.innerPortDepth, 145 + Constants.innerPortDepth, 169 + Constants.innerPortDepth,
          195 + Constants.innerPortDepth, 226 + Constants.innerPortDepth },
      new double[] { 5700, 5500, 5100, 5100, 5100 }, 2);
  private static final PolynomialRegression trenchRegression = new PolynomialRegression(
      new double[] { 172 + Constants.innerPortDepth, 181 + Constants.innerPortDepth, 203 + Constants.innerPortDepth,
          226 + Constants.innerPortDepth, 244 + Constants.innerPortDepth, 263 + Constants.innerPortDepth,
          286 + Constants.innerPortDepth },
      new double[] { 6300, 6100, 6000, 6000, 5900, 6050, 6050 }, 2);

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
    boolean useInnerPort = PointAtTargetWithOdometry.useInnerPort(position);
    double distance = position.getDistance(
        useInnerPort ? PointAtTargetWithOdometry.innerPortTranslation : PointAtTargetWithOdometry.outerPortTranslation);

    // Update hood position
    if (autoHood) {
      double wallFrontLineTransition, frontLineBackLineTransition, backLineTrenchTransition;
      switch (shooterHood.getTargetPosition()) {
        case WALL:
          wallFrontLineTransition = maxWallDistance;
          frontLineBackLineTransition = maxFrontLineDistance;
          backLineTrenchTransition = minTrenchDistance;
          break;
        case FRONT_LINE:
          wallFrontLineTransition = minFrontLineDistance;
          frontLineBackLineTransition = maxFrontLineDistance;
          backLineTrenchTransition = minTrenchDistance;
          break;
        case BACK_LINE:
          wallFrontLineTransition = maxWallDistance;
          frontLineBackLineTransition = minBackLineDistance;
          backLineTrenchTransition = maxBackLineDistance;
          break;
        case TRENCH:
          wallFrontLineTransition = maxWallDistance;
          frontLineBackLineTransition = minBackLineDistance;
          backLineTrenchTransition = minTrenchDistance;
          break;
        case UNKNOWN:
        default:
          wallFrontLineTransition = (maxWallDistance + minFrontLineDistance) / 2;
          frontLineBackLineTransition = (maxFrontLineDistance + minBackLineDistance) / 2;
          backLineTrenchTransition = (maxBackLineDistance + minTrenchDistance) / 2;
          break;
      }
      if (distance < wallFrontLineTransition) {
        shooterHood.setTargetPosition(HoodPosition.WALL);
      } else if (distance < frontLineBackLineTransition) {
        shooterHood.setTargetPosition(HoodPosition.FRONT_LINE);
      } else if (distance < backLineTrenchTransition) {
        shooterHood.setTargetPosition(HoodPosition.BACK_LINE);
      } else {
        shooterHood.setTargetPosition(HoodPosition.TRENCH);
      }
    }

    // Update flywheel speed
    double predictedSpeed;
    switch (shooterHood.getTargetPosition()) {
      case WALL:
        if (useInnerPort) {
          predictedSpeed = wallInnerRegression.predict(distance);
        } else {
          predictedSpeed = wallOuterRegression.predict(distance);
        }
        break;
      case FRONT_LINE:
        predictedSpeed = frontLineRegression.predict(distance);
        break;
      case BACK_LINE:
        predictedSpeed = backLineRegression.predict(distance);
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
