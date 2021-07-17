/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

  private static final int latencyDataPoints = 50; // How many historical data points to keep

  private boolean enableLogging = true;
  private static final String logFolder = "/media/sda1/";
  private static final String logTitle = "'OdometryLog'_yy-MM-dd_HH-mm-ss'.csv'";

  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private DifferentialDriveOdometry driveOdometry;
  private LatencyData xData = new LatencyData(latencyDataPoints);
  private LatencyData yData = new LatencyData(latencyDataPoints);

  private double baseLeftDistance;
  private double baseRightDistance;

  FileWriter csvWriter;

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
    Pose2d pose = updateOdometry();
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Pose x", pose.getTranslation().getX());
      SmartDashboard.putNumber("Pose y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Pose angle", pose.getRotation().getDegrees());
    }

    if (enableLogging) {
      if (csvWriter != null) { // Check if log file ready
        try {
          Pose2d currentPose = getCurrentPose();
          csvWriter.write(String.format("%.4f", Timer.getFPGATimestamp()) + ",");
          csvWriter.write(DriverStation.getInstance().isEnabled() ? "1," : "0,");
          csvWriter.write(String.format("%.2f", currentPose.getX()) + ",");
          csvWriter.write(String.format("%.2f", currentPose.getY()) + ",");
          csvWriter.write(String.format("%.2f", currentPose.getRotation().getDegrees()) + "\n");
        } catch (IOException e) {
          DriverStation.reportWarning("Failed to log odometry values.", false);
        }
      } else {

        // Open log file after time is retrieved from DS
        if (DriverStation.getInstance().getAlliance() != Alliance.Invalid
            && System.currentTimeMillis() > 946702800000L) {
          String logPath = logFolder + new SimpleDateFormat(logTitle).format(new Date());
          try {
            new File(logPath).createNewFile();
            csvWriter = new FileWriter(logPath);
            csvWriter.write(DriverStation.getInstance().getAlliance().name() + "\n");
            csvWriter.write("Timestamp,Enabled,X,Y,Rotation\n");
            System.out.println("Successfully opened log file '" + logPath + "'");
          } catch (IOException e) {
            DriverStation.reportWarning("Failed to open log file '" + logPath + "'", false);
            enableLogging = false;
          }
        }
      }
    }
  }

  private Pose2d updateOdometry() {
    Pose2d pose = driveOdometry.update(getCurrentRotation(), driveTrain.getDistanceLeft() - baseLeftDistance,
        driveTrain.getDistanceRight() - baseRightDistance);
    Translation2d translation = pose.getTranslation();
    xData.addDataPoint(translation.getX());
    yData.addDataPoint(translation.getY());
    return pose;
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
   * Sets the robot's current position to the given Translation2d.
   * 
   * @param position The position (translation only)
   */
  public void setPosition(Translation2d position) {
    driveOdometry.resetPosition(new Pose2d(position, getCurrentPose().getRotation()), getCurrentRotation());
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
    // Since this resets the odometry state make sure the numbers that are not being
    // changed are up to date to avoid losing changes that happened in between the
    // last update and this method call.
    updateOdometry();
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
   * Adjusts the robot's position using the provided x at the timestamp given. The
   * data since then is kept and the y and rotation components of the pose is
   * unchanged.
   * 
   * @param x         The x coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setX(double x, double timestamp) {
    updateOdometry();
    xData.addCorrectedData(x, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(xData.getCurrentPoint(), getCurrentPose().getTranslation().getY(), getCurrentPose().getRotation()),
        getCurrentRotation());
    resetBaseDistances();
  }

  /**
   * Adjusts the robot's position using the provided y at the timestamp given. The
   * data since then is kept and the x and rotation components of the pose is
   * unchanged.
   * 
   * @param y         The y coordinate
   * @param timestamp The timestamp the data is from (FPGA time)
   */
  public void setY(double y, double timestamp) {
    updateOdometry();
    yData.addCorrectedData(y, timestamp);
    driveOdometry.resetPosition(
        new Pose2d(getCurrentPose().getTranslation().getX(), yData.getCurrentPoint(), getCurrentPose().getRotation()),
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
    updateOdometry();
    Translation2d currentTranslation = getCurrentPose().getTranslation();
    driveOdometry.resetPosition(new Pose2d(currentTranslation, rotation), getCurrentRotation());
    resetBaseDistances();
  }
}
