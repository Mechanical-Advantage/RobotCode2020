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
  private static final double minFrontLineDistance = 105; // inches to center of robot
  private static final double maxFrontLineDistance = 185; // inches to center of robot
  private static final double minBackLineDistance = 180; // inches to center of robot
  private static final double maxBackLineDistance = 260; // inches to center of robot
  private static final double minTrenchDistance = 220; // inches to center of robot
  private static final double maxFlywheelSpeed = 7000; // RPM

  private static final PolynomialRegression wallRegression = new PolynomialRegression(
      new double[] { Constants.innerPortDepth + 26, Constants.innerPortDepth + 30, Constants.innerPortDepth + 36,
          Constants.innerPortDepth + 42, Constants.innerPortDepth + 48, Constants.innerPortDepth + 54,
          Constants.innerPortDepth + 60, Constants.innerPortDepth + 66, Constants.innerPortDepth + 72,
          Constants.innerPortDepth + 78 },
      new double[] { 6500, 5500, 3300, 3100, 3100, 3100, 3100, 3100, 3150, 5000 }, 4, "x");
  private static final PolynomialRegression frontLineRegression = new PolynomialRegression(
      new double[] { Constants.innerPortDepth + 78, Constants.innerPortDepth + 90, Constants.innerPortDepth + 102,
          Constants.innerPortDepth + 114, Constants.innerPortDepth + 126, Constants.innerPortDepth + 138,
          Constants.innerPortDepth + 150, Constants.innerPortDepth + 156 },
      new double[] { 3800, 3500, 3550, 3600, 3650, 3700, 3750, 3800 }, 4, "x");
  private static final PolynomialRegression backLineRegression = new PolynomialRegression(
      new double[] { Constants.innerPortDepth + 138, Constants.innerPortDepth + 144, Constants.innerPortDepth + 156,
          Constants.innerPortDepth + 168, Constants.innerPortDepth + 180, Constants.innerPortDepth + 192,
          Constants.innerPortDepth + 204, Constants.innerPortDepth + 216, Constants.innerPortDepth + 228,
          Constants.innerPortDepth + 240 },
      new double[] { 6500, 6000, 5600, 5600, 5600, 5600, 5600, 5600, 5600, 5200 }, 3, "x");
  private static final PolynomialRegression trenchRegression = new PolynomialRegression(
      new double[] { Constants.innerPortDepth + 180, Constants.innerPortDepth + 186, Constants.innerPortDepth + 192,
          Constants.innerPortDepth + 198, Constants.innerPortDepth + 204, Constants.innerPortDepth + 216,
          Constants.innerPortDepth + 228, Constants.innerPortDepth + 240, Constants.innerPortDepth + 252,
          Constants.innerPortDepth + 264, Constants.innerPortDepth + 276 },
      new double[] { 6800, 6600, 6600, 6600, 6600, 6200, 6200, 6400, 6300, 6300, 6300 }, 2, "x");

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
      predictedSpeed = wallRegression.predict(distance);
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
