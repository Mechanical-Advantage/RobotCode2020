package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.subsystems.drive.DriveTrainBase.DriveGear;
import frc.robot.util.UtilFunctions;

/**
 * Drive the specified distance on the specified heading
 * 
 * Does not turn first, really use it to drive straight
 * 
 * Also is not good if the robot is strongly knocked off course (like being
 * kicked)
 */
public class DriveDistanceOnHeading extends CommandBase {

  private double kPDistance;
  private double kIDistance;
  private double kDDistance;
  private double toleranceInches;
  static final int toleranceValuesToAverageDistance = 10;

  private double kPAngle;
  private double kIAngle;
  private double kDAngle;
  private double toleranceDegrees;
  static final int toleranceValuesToAverageAngle = 10;
  // The maximum percent speed to use to correct angle
  private double turnCorrectionAmount = 0.2;

  // PID output will be limited to negative to positive this.
  private double maxOutput = 0.9;
  // Limit change in one iteration to this - % of max output
  private double maxChange = 0.03;
  // If robot does not move for this many cycles, give up
  private static final int stuckCycleThreshold = 50;
  // The maximum distance the robot can move in the number of cycles above to be
  // considered stuck
  private static final double stickDistance = 0.5;

  private DriveGear gear;

  private double maxOutputVelocityChange = maxOutput * maxChange;
  private PIDController distanceController;
  private PIDController turnController;
  private LinearFilter distanceMovingAverage;
  private LinearFilter angleMovingAverage;
  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private double targetDistance;
  private double targetAngle;
  private boolean useStartingYaw;
  private boolean resetCompletedDistance;
  private boolean resetStartedDistance;
  private double lastOutputDistance;
  private double lastDistance;
  private int stuckCycles;

  /**
   * Construct a new DriveDistanceOnHeading command using the starting heading
   * 
   * @param distance Distance to drive
   */
  public DriveDistanceOnHeading(DriveTrainBase driveTrain, AHRS ahrs, double distance) {
    this(driveTrain, ahrs, distance, 0);
    useStartingYaw = true;
  }

  /**
   * Construct a new DriveDistanceOnHeading command using the starting heading
   * 
   * @param distance        Distance to drive
   * @param toleranceInches How many inches off the end distance can be (pass 0 to
   *                        use default)
   * @param maxChange       Maximum percent velocity change per cycle (pass 0 to
   *                        use default)
   * @param maxOutput       Maximum percent velocity (pass 0 to use default)
   */
  public DriveDistanceOnHeading(DriveTrainBase driveTrain, AHRS ahrs, double distance, double toleranceInches,
      double maxChange, double maxOutput) {
    this(driveTrain, ahrs, distance, 0, toleranceInches, maxChange, maxOutput);
    useStartingYaw = true;
  }

  /**
   * Construct a new DriveDistanceOnHeading command with a defined heading
   * 
   * @param distance        Distance to drive
   * @param heading         Heading to try to follow
   * @param toleranceInches How many inches off the end distance can be (pass 0 to
   *                        use default)
   * @param maxChange       Maximum percent velocity change per cycle (pass 0 to
   *                        use default)
   * @param maxOutput       Maximum percent velocity (pass 0 to use default)
   */
  public DriveDistanceOnHeading(DriveTrainBase driveTrain, AHRS ahrs, double distance, double heading,
      double toleranceInches, double maxChange, double maxOutput) {
    this(driveTrain, ahrs, distance, heading);
    boolean maxPercentsChanged = false;
    if (maxOutput != 0) {
      this.maxOutput = maxOutput;
      turnCorrectionAmount *= maxOutput; // If a different max output is specified, calculate turn range as
      // percent of that
      maxPercentsChanged = true;
    }
    if (maxChange != 0) {
      this.maxChange = maxChange;
      maxPercentsChanged = true;
    }
    if (maxPercentsChanged) {
      maxOutputVelocityChange = maxOutput * maxChange;
    }
    if (toleranceInches != 0) {
      this.toleranceInches = toleranceInches;
    }
  }

  /**
   * Construct a new DriveDistanceOnHeading command with a defined heading
   * 
   * @param distance Distance to drive
   * @param heading  Heading to try to follow
   */
  public DriveDistanceOnHeading(DriveTrainBase driveTrain, AHRS ahrs, double distance, double heading) {
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    addRequirements(driveTrain);
    targetDistance = distance;
    targetAngle = UtilFunctions.boundHalfDegrees(heading);
    useStartingYaw = false;
    switch (Constants.getRobot()) {
    case REBOT:
      kPDistance = 0.032;
      kIDistance = 0.000000;
      kDDistance = 0;
      toleranceInches = 0.5;
      kPAngle = 0.05;
      kIAngle = 0;
      kDAngle = 0;
      toleranceDegrees = 1;
      break;
    case ORIGINAL_ROBOT_2018:
      kPDistance = 0.02;
      kIDistance = 0.000000;
      kDDistance = 0;
      toleranceInches = 0.5;
      kPAngle = 0.05;
      kIAngle = 0;
      kDAngle = 0;
      toleranceDegrees = 1;
      gear = DriveGear.HIGH;
      break;
    case NOTBOT:
      kPDistance = 0.017;
      kIDistance = 0;
      kDDistance = 0;
      toleranceInches = 0.5;
      kPAngle = 0.07;
      kIAngle = 0;
      kDAngle = 0;
      toleranceDegrees = 0.5;
      break;
    case ROBOT_2019:
      kPDistance = 0.014;
      kIDistance = 0.000000;
      kDDistance = 0;
      toleranceInches = 0.5;
      kPAngle = 0.05;
      kIAngle = 0;
      kDAngle = 0;
      toleranceDegrees = 1;
      break;
    case ROBOT_2020:
    case ROBOT_2020_DRIVE:
      kPDistance = 0.015;
      kIDistance = 0;
      kDDistance = 0;
      toleranceInches = 0.5;
      kPAngle = 0.05;
      kIAngle = 0;
      kDAngle = 0;
      toleranceDegrees = 1;
    default:
      break;
    }
    distanceController = new PIDController(kPDistance, kIDistance, kDDistance);
    turnController = new PIDController(kPAngle, kIAngle, kDAngle);
    distanceController.setTolerance(toleranceInches);
    distanceController.setSetpoint(targetDistance);
    turnController.setTolerance(toleranceDegrees);
    turnController.enableContinuousInput(-180, 180);
    distanceMovingAverage = LinearFilter.movingAverage(toleranceValuesToAverageDistance);
    angleMovingAverage = LinearFilter.movingAverage(toleranceValuesToAverageAngle);
  }

  @Override
  public void initialize() {
    if (driveTrain.isDualGear()) {
      driveTrain.switchGear(gear);
    }
    if (useStartingYaw) {
      targetAngle = ahrs.getYaw();
    }

    // For reseting encoders, wheels need to not be moving, but they could be
    // commanded to move from a previous command
    driveTrain.stop();

    distanceController.reset();
    turnController.reset();
    distanceMovingAverage.reset();
    angleMovingAverage.reset();
    turnController.setSetpoint(targetAngle);
    lastOutputDistance = 0;
    resetCompletedDistance = false;
    resetStartedDistance = false;
    stuckCycles = 0;
    lastDistance = 0;
  }

  /**
   * Processes the input to take the maximum output (from maxOutput and
   * turnCorrectionAmount) into account and limits change by maxVelocityChange
   * 
   * @param currentOutput The current raw PID controller output (-1 to 1)
   * @param lastOutput    The previous processed output
   * @return The new processed output
   */
  private double calcNewVelocity(double currentOutput, double lastOutput) {
    double targetOutput = currentOutput * (maxOutput - turnCorrectionAmount);
    if (Math.abs(lastOutput - targetOutput) > maxOutputVelocityChange) {
      if (lastOutput < targetOutput) {
        targetOutput = lastOutput + maxOutputVelocityChange;
      } else {
        targetOutput = lastOutput - maxOutputVelocityChange;
      }
    }
    return targetOutput;
  }

  @Override
  public void execute() {
    if (!resetStartedDistance && Math.abs(driveTrain.getVelocityLeft()) < 1
        && Math.abs(driveTrain.getVelocityRight()) < 1) {
      driveTrain.resetPosition();
      resetStartedDistance = true;
    }
    if (resetStartedDistance && Math.abs(driveTrain.getDistanceLeft()) < 0.1
        && Math.abs(driveTrain.getDistanceRight()) < 0.1) {
      resetCompletedDistance = true;
    }
    if (resetCompletedDistance) {
      // The PID controller outputs are clamped to -1 to 1
      double outputVelocity = calcNewVelocity(MathUtil.clamp(distanceController.calculate(getAverageDistance()), -1, 1),
          lastOutputDistance);
      lastOutputDistance = outputVelocity;
      double outputTurnVelocity = MathUtil.clamp(turnController.calculate(ahrs.getYaw()), -1, 1) * turnCorrectionAmount;
      // subtract from right side, add to left side (drive left on positive)
      driveTrain.drive(outputVelocity + outputTurnVelocity, outputVelocity - outputTurnVelocity);
      if (Constants.tuningMode) {
        SmartDashboard.putNumber("Current Distance", getAverageDistance());
        SmartDashboard.putNumber("Target Distance", targetDistance);
        SmartDashboard.putNumber("Center Velocity", outputVelocity);
        SmartDashboard.putNumber("Left Velocity", outputVelocity + outputTurnVelocity);
        SmartDashboard.putNumber("Right Velocity", outputVelocity - outputTurnVelocity);
        SmartDashboard.putNumber("Turn Velocity", outputTurnVelocity);
        SmartDashboard.putNumber("Target yaw", targetAngle);
        SmartDashboard.putNumber("Current Angle", ahrs.getYaw());
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(getAverageDistance() - lastDistance) <= stickDistance) {
      stuckCycles++;
    } else {
      stuckCycles = 0;
      lastDistance = getAverageDistance();
    }
    // Both controllers must be within their tolerance and the moving averages must
    // be as well
    return resetCompletedDistance && ((distanceController.atSetpoint() && turnController.atSetpoint()
        && (Math.abs(distanceMovingAverage.calculate(getAverageDistance()) - targetDistance) < toleranceInches)
        && (Math.abs(angleMovingAverage.calculate(ahrs.getYaw()) - targetAngle) < toleranceDegrees))
        || stuckCycles >= stuckCycleThreshold);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  private double getAverageDistance() {
    return (driveTrain.getDistanceLeft() + driveTrain.getDistanceRight()) / 2;
  }
}
