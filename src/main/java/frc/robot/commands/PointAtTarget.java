/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;
import frc.robot.util.LatencyData;

public class PointAtTarget extends CommandBase {

  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private LimelightInterface limelight;
  private LatencyData angleData;
  private PIDController spinController;
  private double kP;
  private double kI;
  private double kD;
  private double toleranceDegrees;
  // The average of this many of the most recent points must be within tolerance
  // to end
  private int toleranceValuesToAverage = 1;
  private DriveGear gear;
  private PIDController turnController;
  private LinearFilter movingAverageFilter;
  private double lastAngle;
  private double targetAngle;

  /**
   * Creates a new PointAtTarget.
   */
  public PointAtTarget(DriveTrainBase driveTrain, LimelightInterface limelight, AHRS ahrs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    addRequirements(limelight);
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    this.limelight = limelight;
    switch (Constants.getRobot()) {
    case REBOT:
      kP = 0.0077; // 0.008
      kI = 0;
      kD = 0.0137; // 0.014
      toleranceDegrees = 1.0;
      toleranceValuesToAverage = 10;
      break;
    case ORIGINAL_ROBOT_2018:
      kP = 0.007;
      kI = 0;
      kD = 0.02;
      toleranceDegrees = 1;
      toleranceValuesToAverage = 3;
      gear = DriveGear.LOW;
      break;
    case NOTBOT:
      // This has a slower update rate (0.05 sec) before so these gains are probably
      // wrong (too high)
      kP = 0.01;
      kI = 0;
      kD = 0.003;
      toleranceDegrees = 1.0;
      toleranceValuesToAverage = 10;
      break;
    case ROBOT_2019:
      kP = 0.007;
      kI = 0;
      kD = 0.00015; // 0.015 in old command
      toleranceDegrees = 1;
      toleranceValuesToAverage = 3;
      break;
    default:
      break;
    }
    turnController = new PIDController(kP, kI, kD);
    turnController.setTolerance(toleranceDegrees);
    turnController.enableContinuousInput(-180, 180);
    movingAverageFilter = LinearFilter.movingAverage(toleranceValuesToAverage);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driveTrain.isDualGear()) {
      driveTrain.switchGear(gear);
    }
    turnController.reset();
    turnController.setSetpoint(targetAngle);
    movingAverageFilter.reset();
    angleData.clear();
    lastAngle = ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleChange = ahrs.getAngle() - lastAngle;
    angleData.addDataPoint(angleData.getCurrentPoint() + angleChange);
    if (limelight.hasValidTarget()) {
      angleData.addCorrectedData(limelight.getTargetHorizAngle(),
          Timer.getFPGATimestamp() - (limelight.getLatency() / 1000));
    }
    double outputVelocity = turnController.calculate(angleData.getCurrentPoint());
    driveTrain.drive(outputVelocity, outputVelocity * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The moving average and current value must both be within tolerance
    return turnController.atSetpoint()
        && Math.abs(movingAverageFilter.calculate(angleData.getCurrentPoint()) - targetAngle) <= toleranceDegrees;
  }
}
