// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.TunableNumber;

public class DriveToTarget extends CommandBase {

  private static final Transform2d fieldToInnerPort = new Transform2d(
      new Translation2d(Constants.fieldLength + Constants.innerPortDepth, Constants.visionTargetHorizDist * -1),
      new Rotation2d());
  private static final Transform2d fieldToOuterPort = new Transform2d(
      new Translation2d(Constants.fieldLength, Constants.visionTargetHorizDist * -1), new Rotation2d());

  private static final double maxVelocity = 0.85;
  private static final double maxAcceleration = 250.0 / 150.0; // acceleration over max velocity (%/s^2)
  private static final double angularTolerance = 1;
  private static final double toleranceTime = 0.15;

  private final DriveTrainBase driveTrain;
  private final RobotOdometry odometry;
  private final double xPosition;
  private final TunableNumber angularKp = new TunableNumber("DriveToTarget/AngularKp");
  private final TunableNumber angularKi = new TunableNumber("DriveToTarget/AngularKi");
  private final TunableNumber angularKd = new TunableNumber("DriveToTarget/AngularKd");
  private final PIDController angularController;
  private final Timer accelerationTimer = new Timer();
  private final Timer toleranceTimer = new Timer();
  private boolean linearReady = false;

  /** Creates a new DriveToPoint. */
  public DriveToTarget(DriveTrainBase driveTrain, RobotOdometry odometry, double xPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.odometry = odometry;
    this.xPosition = xPosition;
    addRequirements(driveTrain);

    switch (Constants.getRobot()) {
      case ROBOT_2020:
      case ROBOT_2020_DRIVE:
        angularKp.setDefault(0.016);
        angularKi.setDefault(0);
        angularKd.setDefault(0.0003);
        break;
      default:
        angularKp.setDefault(0);
        angularKi.setDefault(0);
        angularKd.setDefault(0);
        break;
    }
    angularController = new PIDController(angularKp.get(), angularKi.get(), angularKd.get());
    angularController.setTolerance(angularTolerance);
    angularController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearReady = false;
    angularController.reset();
    accelerationTimer.reset();
    accelerationTimer.start();
    toleranceTimer.reset();
    toleranceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("DriveToTarget/AngularError", angularController.getPositionError());
      angularController.setPID(angularKp.get(), angularKi.get(), angularKd.get());
    }
    Pose2d fieldToVehicle = odometry.getCurrentPose();

    // Calculate linear speed
    double linearSpeed;
    if (fieldToVehicle.getX() < xPosition) {
      linearSpeed = maxVelocity;
      accelerationTimer.reset();
    } else {
      linearSpeed = maxVelocity - (accelerationTimer.get() * maxAcceleration);
      linearSpeed = linearSpeed < 0 ? 0 : linearSpeed;
    }
    linearReady = linearSpeed == 0;

    // Calculate angular speed
    Translation2d vehicleToTarget = (Constants.flatTarget ? fieldToOuterPort : fieldToInnerPort).getTranslation()
        .minus(fieldToVehicle.getTranslation());
    Rotation2d vehicleToInnerPortRotation = new Rotation2d(vehicleToTarget.getX(), vehicleToTarget.getY());
    angularController.setSetpoint(vehicleToInnerPortRotation.getDegrees());
    double angularSpeed = angularController.calculate(fieldToVehicle.getRotation().getDegrees());
    if (!angularController.atSetpoint()) {
      toleranceTimer.reset();
    }

    driveTrain.drive(linearSpeed - angularSpeed, linearSpeed + angularSpeed);
  }

  public boolean angularReady() {
    return toleranceTimer.hasElapsed(toleranceTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    toleranceTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return linearReady && angularReady();
  }
}