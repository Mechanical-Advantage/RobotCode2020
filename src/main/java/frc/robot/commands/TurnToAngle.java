package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;
import frc.robot.util.UtilFunctions;

/**
 * Turns the specified number of degrees, -180 to 180.
 */
public class TurnToAngle extends CommandBase {

  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private double kP;
  private double kI;
  private double kD;
  private double toleranceDegrees;
  // The average of this many of the most recent points must be within tolerance
  // to end
  private int toleranceValuesToAverage;
  private DriveGear gear;
  private PIDController turnController;
  private LinearFilter movingAverageFilter;
  private double targetAngle;
  private boolean absoluteAngle;

  public TurnToAngle(DriveTrainBase driveTrain, AHRS ahrs, double angle, boolean absoluteAngle, double tolerance) {
    this(driveTrain, ahrs, angle, absoluteAngle);
    toleranceDegrees = tolerance;
  }

  public TurnToAngle(DriveTrainBase driveTrain, AHRS ahrs, double angle, double tolerance) {
    this(driveTrain, ahrs, angle);
    toleranceDegrees = tolerance;
  }

  public TurnToAngle(DriveTrainBase driveTrain, AHRS ahrs, double angle) {
    this(driveTrain, ahrs, angle, false);
  }

  public TurnToAngle(DriveTrainBase driveTrain, AHRS ahrs, double angle, boolean absoluteAngle) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    this.absoluteAngle = absoluteAngle;

    // limit input to -179 to 180
    targetAngle = UtilFunctions.boundHalfDegrees(angle);
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
      kD = 0.015;
      toleranceDegrees = 1;
      toleranceValuesToAverage = 3;
      break;
    default:
      break;
    }
    movingAverageFilter = LinearFilter.movingAverage(toleranceValuesToAverage);
  }

  @Override
  public void initialize() {
    if (driveTrain.isDualGear()) {
      driveTrain.switchGear(gear);
    }
    turnController = new PIDController(kP, kI, kD);
    turnController.setTolerance(toleranceDegrees);
    turnController.enableContinuousInput(-180, 180);

    double currentTargetAngle = UtilFunctions
        .boundHalfDegrees(absoluteAngle ? targetAngle : ahrs.getYaw() + targetAngle);
    turnController.setSetpoint(currentTargetAngle);
    movingAverageFilter.reset();
  }

  @Override
  public void execute() {
    double outputVelocity = turnController.calculate(ahrs.getYaw());
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Angle", ahrs.getAngle());
      SmartDashboard.putNumber("Rate", ahrs.getRate());
      SmartDashboard.putNumber("Yaw", ahrs.getYaw());
      SmartDashboard.putNumber("Current Output", outputVelocity);
    }
    driveTrain.drive(outputVelocity, outputVelocity * -1);
  }

  @Override
  public boolean isFinished() {
    // The moving average and current value must both be within tolerance
    return turnController.atSetpoint()
        && Math.abs(movingAverageFilter.calculate(ahrs.getYaw()) - targetAngle) <= toleranceDegrees;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }
}
