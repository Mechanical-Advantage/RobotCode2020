// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.TunableNumber;

public class PointAtTargetWithOdometry extends CommandBase {
  public static final Translation2d innerPortTranslation = new Translation2d(
      Constants.fieldLength + Constants.innerPortDepth, Constants.visionTargetHorizDist * -1);
  public static final Translation2d outerPortTranslation = new Translation2d(Constants.fieldLength,
      Constants.visionTargetHorizDist * -1);
  private static final double innerPortMaxDegrees = 12; // If angle outside this value, aim at outer

  private final RobotOdometry odometry;
  private final LimelightInterface limelight;
  private final DriveTrainBase driveTrain;
  private final TunableNumber kP = new TunableNumber("PointAtTarget/kP");
  private final TunableNumber kI = new TunableNumber("PointAtTarget/kI");
  private final TunableNumber kD = new TunableNumber("PointAtTarget/kD");
  private final TunableNumber integralMaxError = new TunableNumber("PointAtTarget/IntegralMaxError");
  private final TunableNumber minVelocity = new TunableNumber("PointAtTarget/MinVelocity");
  private final TunableNumber toleranceDegrees = new TunableNumber("PointAtTarget/ToleranceDegrees");
  private final TunableNumber toleranceTime = new TunableNumber("PointAtTarget/ToleranceTime");

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
        kP.setDefault(0.018);
        kI.setDefault(0.005);
        kD.setDefault(0.001);
        integralMaxError.setDefault(4);
        minVelocity.setDefault(0.02);
        toleranceDegrees.setDefault(1.5);
        toleranceTime.setDefault(0.1);
        break;
      default:
        kP.setDefault(0);
        kI.setDefault(0);
        kD.setDefault(0);
        integralMaxError.setDefault(10);
        minVelocity.setDefault(0);
        toleranceDegrees.setDefault(1);
        toleranceTime.setDefault(0.25);
        break;
    }
    turnController = new PIDController(kP.get(), 0, kD.get());
    turnController.setTolerance(toleranceDegrees.get());
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
    // Update tunable numbers
    if (Constants.tuningMode) {
      turnController.setP(kP.get());
      turnController.setD(kD.get());
      turnController.setTolerance(toleranceDegrees.get());
    }

    // Update setpoint
    Pose2d fieldToVehicle = odometry.getCurrentPose();
    boolean useInner = useInnerPort(fieldToVehicle.getTranslation());
    Translation2d targetPosition = useInner ? innerPortTranslation : outerPortTranslation;
    Translation2d targetRelative = targetPosition.minus(fieldToVehicle.getTranslation());
    Rotation2d targetRotation = new Rotation2d(targetRelative.getX(), targetRelative.getY());
    turnController.setSetpoint(targetRotation.getDegrees());

    // Check if in tolerance
    if (!turnController.atSetpoint()) {
      toleranceTimer.reset();
    }

    // Update output speeds
    if (Math.abs(turnController.getPositionError()) < integralMaxError.get()) {
      turnController.setI(kI.get());
    } else {
      turnController.setI(0);
    }
    double output = turnController.calculate(fieldToVehicle.getRotation().getDegrees());
    if (Math.abs(output) < minVelocity.get()) {
      output = Math.copySign(minVelocity.get(), output);
    }
    driveTrain.drive(output * -1, output);

    // Log in tuning mode
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("PointAtTarget/ErrorDegrees", turnController.getPositionError());
      SmartDashboard.putBoolean("PointAtTarget/UsingInner", useInner);
    }
  }

  /**
   * Determines whether to aim at the inner or outer port depending on angle and
   * type of port
   */
  public static boolean useInnerPort(Translation2d currentPosition) {
    return false;
    /*
     * Translation2d innerPortRelative =
     * innerPortTranslation.minus(currentPosition); Rotation2d innerPortRotation =
     * new Rotation2d(innerPortRelative.getX(), innerPortRelative.getY());
     * 
     * return Math.abs(innerPortRotation.getDegrees()) < innerPortMaxDegrees &&
     * !Constants.flatTarget;
     */
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
    return toleranceTimer.hasElapsed(toleranceTime.get()) && odometry.isUsingVision();
  }
}
