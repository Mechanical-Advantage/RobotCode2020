/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.LatencyData;

public class RobotOdometry extends SubsystemBase {

  // How many historical data points to keep. Multiply by 20ms to get time
  private static final int latencyDataPoints = 20;

  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private DifferentialDriveOdometry driveOdometry;
  private LatencyData xData = new LatencyData(latencyDataPoints);
  private LatencyData yData = new LatencyData(latencyDataPoints);

  private double baseLeftDistance;
  private double baseRightDistance;

  /**
   * Creates a new RobotOdometry. Commands can require this subsystem to prevent
   * the position from being changed by anything other than normal dead reckoning
   * while the command is running.
   * 
   * Initial position is (0,0) and 0 degrees
   */
  public RobotOdometry(DriveTrainBase driveTrain, AHRS ahrs) {
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    driveOdometry = new DifferentialDriveOdometry(getCurrentRotation());
    resetBaseDistances();
  }

  @Override
  public void periodic() {
    Pose2d pose = driveOdometry.update(getCurrentRotation(), driveTrain.getDistanceLeft() - baseLeftDistance,
        driveTrain.getDistanceRight() - baseRightDistance);
    Translation2d translation = pose.getTranslation();
    xData.addDataPoint(translation.getX());
    yData.addDataPoint(translation.getY());
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Pose x", translation.getX());
      SmartDashboard.putNumber("Pose y", translation.getY());
      SmartDashboard.putNumber("Pose angle", pose.getRotation().getDegrees());
    }
  }

  private Rotation2d getCurrentRotation() {
    return Rotation2d.fromDegrees(ahrs.getYaw() * -1);
  }

  /**
   * Resets baseLeftDistance and baseRightDistance to the current position so
   * distances reported to the odometry class are 0.
   */
  private void resetBaseDistances() {
    baseLeftDistance = driveTrain.getDistanceLeft();
    baseRightDistance = driveTrain.getDistanceRight();
  }

  /**
   * Gets the robot's current pose.
   */
  public Pose2d getCurrentPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
   * Sets the robot's current pose to the given x/y/angle.
   * 
   * @param x     The x coordinate
   * @param y     The y coordinate
   * @param angle The rotation component
   */
  public void setPosition(double x, double y, Rotation2d angle) {
    setPosition(new Pose2d(x, y, angle));
  }

  /**
   * Sets the robot's current pose to the given Pose2d.
   * 
   * @param position The position (both translation and rotation)
   */
  public void setPosition(Pose2d position) {
    driveOdometry.resetPosition(position, getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Adjusts the robot's position using the provided translation at the timestamp
   * given. The data since then is kept and the rotation component of the pose is
   * unchanged.
   * 
   * @param position  The translation component of the position
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setPosition(Translation2d position, double timestamp) {
    setPosition(position.getX(), position.getY(), timestamp);
  }

  /**
   * Adjusts the robot's position using the provided x and y at the timestamp
   * given. The data since then is kept and the rotation component of the pose is
   * unchanged.
   * 
   * @param x         The x coordinate
   * @param y         The y coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setPosition(double x, double y, double timestamp) {
    // This uses xData and yData to perform latency correction on the x/y portion of
    // the provided position
    xData.addCorrectedData(x, timestamp);
    yData.addCorrectedData(y, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(xData.getCurrentPoint(), yData.getCurrentPoint(), getCurrentPose().getRotation()),
        getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Sets the robot's current rotation without affecting the translation component
   * of the pose.
   * 
   * @param rotation The rotation component of the pose
   */
  public void setRotation(Rotation2d rotation) {
    Translation2d currentTranslation = getCurrentPose().getTranslation();
    driveOdometry.resetPosition(new Pose2d(currentTranslation, rotation), getCurrentRotation());
    resetBaseDistances();
  }
}
