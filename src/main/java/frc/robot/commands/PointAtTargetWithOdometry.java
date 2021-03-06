// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.drive.DriveTrainBase;

public class PointAtTargetWithOdometry extends CommandBase {
  private static final Transform2d fieldToInnerPort = new Transform2d(
      new Translation2d(Constants.fieldLength + Constants.innerPortDepth, Constants.visionTargetHorizDist * -1),
      new Rotation2d());
  private static final Transform2d fieldToOuterPort = new Transform2d(
      new Translation2d(Constants.fieldLength, Constants.visionTargetHorizDist * -1), new Rotation2d());
  private static final double innerPortMaxDegrees = 20; // If angle outside this value, aim at outer
  private static final double minTime = 0.75; // Do not exit until this many seconds have passed (allows Limelight to
                                              // begin writing data)

  private final RobotOdometry odometry;
  private final LimelightInterface limelight;
  private final DriveTrainBase driveTrain;
  private final double kP;
  private final double kI;
  private final double kD;
  private final double minVelocity;
  private final double toleranceDegrees;
  private final double toleranceTime;

  private PIDController turnController;
  private Timer toleranceTimer = new Timer();
  private Timer minTimer = new Timer();

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
        kI = 0;
        kD = 0.0003;
        minVelocity = 0.04;
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
    minTimer.reset();
    minTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update setpoint
    Pose2d fieldToVehicle = odometry.getCurrentPose();

    Translation2d vehicleToInnerPort = fieldToInnerPort.getTranslation().minus(fieldToVehicle.getTranslation());
    Rotation2d vehicleToInnerPortRotation = new Rotation2d(vehicleToInnerPort.getX(), vehicleToInnerPort.getY());

    if (Math.abs(vehicleToInnerPortRotation.getDegrees()) < innerPortMaxDegrees) {
      turnController.setSetpoint(vehicleToInnerPortRotation.getDegrees());
    } else {
      Translation2d vehicleToOuterPort = fieldToOuterPort.getTranslation().minus(fieldToVehicle.getTranslation());
      Rotation2d vehicleToOuterPortRotation = new Rotation2d(vehicleToOuterPort.getX(), vehicleToOuterPort.getY());
      turnController.setSetpoint(vehicleToOuterPortRotation.getDegrees());
    }

    // Check if in tolerance
    if (!turnController.atSetpoint()) {
      toleranceTimer.reset();
    }

    // Update output speeds
    double output = turnController.calculate(fieldToVehicle.getRotation().getDegrees());
    if (Math.abs(output) < minVelocity) {
      output = Math.copySign(minVelocity, output);
    }
    driveTrain.drive(output * -1, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    limelight.setLEDMode(LimelightLEDMode.OFF);
    toleranceTimer.stop();
    minTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toleranceTimer.hasElapsed(toleranceTime) && minTimer.hasElapsed(minTime);
  }
}
