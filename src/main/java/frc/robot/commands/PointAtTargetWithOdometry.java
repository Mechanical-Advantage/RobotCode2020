// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.drive.DriveTrainBase;

public class PointAtTargetWithOdometry extends CommandBase {
  public static final Translation2d innerPortTranslation = new Translation2d(
      Constants.fieldLength + Constants.innerPortDepth, Constants.visionTargetHorizDist * -1);
  public static final Translation2d outerPortTranslation = new Translation2d(Constants.fieldLength,
      Constants.visionTargetHorizDist * -1);
  private static final double innerPortMaxDegrees = 15; // If angle outside this value, aim at outer

  private final RobotOdometry odometry;
  private final LimelightInterface limelight;
  private final DriveTrainBase driveTrain;
  private final double kP;
  private final double kI;
  private final double kD;
  private final double integralMaxError = 10.0;
  private final double minVelocity;
  private final double toleranceDegrees;
  private final double toleranceTime;

  private PIDController turnController;
  private Timer toleranceTimer = new Timer();

  /** Creates a new PointAtTargetWithOdometry. */
  public PointAtTargetWithOdometry(DriveTrainBase driveTrain, RobotOdometry odometry, LimelightInterface limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, limelight);
    this.driveTrain = driveTrain;
    this.odometry = odometry;
    this.limelight = limelight;
    switch (Constants.getRobot()) {
      case ROBOT_2020:
      case ROBOT_2020_DRIVE:
        kP = 0.01;
        kI = 0.007;
        kD = 0.0003;
        minVelocity = 0.045;
        toleranceDegrees = 1;
        toleranceTime = 0.25;
        break;
      default:
        kP = 0;
        kI = 0;
        kD = 0;
        minVelocity = 0;
        toleranceDegrees = 1;
        toleranceTime = 0.25;
        break;
    }
    turnController = new PIDController(kP, kI, kD);
    turnController.setTolerance(toleranceDegrees);
    turnController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.reset();
    limelight.setLEDMode(LimelightLEDMode.ON);
    toleranceTimer.reset();
    toleranceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update setpoint
    Pose2d fieldToVehicle = odometry.getCurrentPose();

    Translation2d targetPosition = useInnerPort(fieldToVehicle.getTranslation()) ? innerPortTranslation
        : outerPortTranslation;
    Translation2d targetRelative = targetPosition.minus(fieldToVehicle.getTranslation());
    Rotation2d targetRotation = new Rotation2d(targetRelative.getX(), targetRelative.getY());
    turnController.setSetpoint(targetRotation.getDegrees());

    // Check if in tolerance
    if (!turnController.atSetpoint()) {
      toleranceTimer.reset();
    }

    // Update output speeds
    if (Math.abs(turnController.getPositionError()) < integralMaxError) {
      turnController.setI(kI);
    } else {
      turnController.setI(0);
    }
    double output = turnController.calculate(fieldToVehicle.getRotation().getDegrees());
    if (Math.abs(output) < minVelocity) {
      output = Math.copySign(minVelocity, output);
    }
    driveTrain.drive(output * -1, output);
  }

  /**
   * Determines whether to aim at the inner or outer port depending on angle and
   * type of port
   */
  public static boolean useInnerPort(Translation2d currentPosition) {
    Translation2d innerPortRelative = innerPortTranslation.minus(currentPosition);
    Rotation2d innerPortRotation = new Rotation2d(innerPortRelative.getX(), innerPortRelative.getY());

    return Math.abs(innerPortRotation.getDegrees()) < innerPortMaxDegrees && !Constants.flatTarget;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    limelight.setLEDMode(LimelightLEDMode.OFF);
    toleranceTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime) && odometry.isUsingVision();
  }
}
