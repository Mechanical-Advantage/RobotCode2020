// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

  private static final double linearTolerance = 6;
  private static final double angularTolerance = 1;
  private static final double toleranceTime = 0.25;

  private final DriveTrainBase driveTrain;
  private final RobotOdometry odometry;
  private final TunableNumber linearKp = new TunableNumber("DriveToTarget/LinearKp");
  private final TunableNumber linearKi = new TunableNumber("DriveToTarget/LinearKi");
  private final TunableNumber linearKd = new TunableNumber("DriveToTarget/LinearKd");
  private final TunableNumber angularKp = new TunableNumber("DriveToTarget/AngularKp");
  private final TunableNumber angularKi = new TunableNumber("DriveToTarget/AngularKi");
  private final TunableNumber angularKd = new TunableNumber("DriveToTarget/AngularKd");
  private final PIDController linearController;
  private final PIDController angularController;
  private final Timer toleranceTimer = new Timer();

  /** Creates a new DriveToPoint. */
  public DriveToTarget(DriveTrainBase driveTrain, RobotOdometry odometry, double xPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.odometry = odometry;
    addRequirements(driveTrain);

    switch (Constants.getRobot()) {
    case ROBOT_2020:
    case ROBOT_2020_DRIVE:
      linearKp.setDefault(0.018);
      linearKi.setDefault(0);
      linearKd.setDefault(0.0008);
      angularKp.setDefault(0.015);
      angularKi.setDefault(0);
      angularKd.setDefault(0.0003);
      break;
    default:
      linearKp.setDefault(0);
      linearKi.setDefault(0);
      linearKd.setDefault(0);
      angularKp.setDefault(0);
      angularKi.setDefault(0);
      angularKd.setDefault(0);
      break;
    }
    linearController = new PIDController(linearKp.get(), linearKi.get(), linearKd.get());
    linearController.setTolerance(linearTolerance);
    linearController.setSetpoint(xPosition);
    angularController = new PIDController(angularKp.get(), angularKi.get(), angularKd.get());
    angularController.setTolerance(angularTolerance);
    angularController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearController.reset();
    angularController.reset();
    toleranceTimer.reset();
    toleranceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("DriveToTarget/LinearError", linearController.getPositionError());
      SmartDashboard.putNumber("DriveToTarget/AngularError", angularController.getPositionError());
      linearController.setPID(linearKp.get(), linearKi.get(), linearKd.get());
      angularController.setPID(angularKp.get(), angularKi.get(), angularKd.get());
    }

    Pose2d fieldToVehicle = odometry.getCurrentPose();
    double linearSpeed = linearController.calculate(fieldToVehicle.getX());

    Translation2d vehicleToTarget = (Constants.flatTarget ? fieldToOuterPort : fieldToInnerPort).getTranslation()
        .minus(fieldToVehicle.getTranslation());
    Rotation2d vehicleToInnerPortRotation = new Rotation2d(vehicleToTarget.getX(), vehicleToTarget.getY());
    angularController.setSetpoint(vehicleToInnerPortRotation.getDegrees());
    double angularSpeed = angularController.calculate(fieldToVehicle.getRotation().getDegrees());

    double linearPower = 1 - (Math.abs(angularController.getPositionError()) / 45);
    if (linearPower < 0) {
      linearPower = 0;
    }
    linearSpeed *= linearPower;
    driveTrain.drive(linearSpeed - angularSpeed, linearSpeed + angularSpeed);

    if (!linearController.atSetpoint() || !angularController.atSetpoint()) {
      toleranceTimer.reset();
    }
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
    return toleranceTimer.hasElapsed(toleranceTime);
  }
}