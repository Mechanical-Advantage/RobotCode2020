/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frckit.vision.DualCornerVisionKinematics;
import frckit.vision.corner.*;

public class LimelightOdometry extends CommandBase {

  private static final boolean alwaysUseFarTarget = false; // For use on half field (means gyro doesn't have to be
                                                           // zeroed)
  private static final double loopCycle = 0.02;

  private static final Transform2d vehicleToCamera = new Transform2d(new Translation2d(1.875, 0.0), new Rotation2d());
  private static final double targetHeight = 98; // to center of target area (hexagon)
  private static final double cameraHeight = 19.75;
  private static final double cameraVertAngle = 29.8; // 0 is straight forward
  private static final int odometryPipeline = 1; // Data will be ignored for other pipelines

  private LimelightInterface limelight;
  private RobotOdometry odometry;

  // Perspective testing
  private final LimelightCornerSupplier cornerSupplier = new LimelightCornerSupplier("limelight");
  private final DualTopCornerSupplier topCornerSupplier = new DualTopCornerSupplier(cornerSupplier);
  private final DualCornerVisionKinematics visionKinematics = new DualCornerVisionKinematics(topCornerSupplier,
      Rotation2d.fromDegrees(cameraVertAngle), cameraHeight, targetHeight);

  private static final int averagingTapsTotal = 25;
  private int averagingTapsCurrent = 0;
  private final LinearFilter xAveraging = LinearFilter.movingAverage(averagingTapsTotal);
  private final LinearFilter yAveraging = LinearFilter.movingAverage(averagingTapsTotal);

  /**
   * Creates a new LimelightOdometry. Note that this command does not require the
   * limelight but does not expect exclusive access. It will not call any set
   * methods and will only use data when the limelight is in a state it can use.
   */
  public LimelightOdometry(LimelightInterface limelight, RobotOdometry odometry) {
    addRequirements(odometry);
    this.limelight = limelight;
    this.odometry = odometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setLEDMode(LimelightLEDMode.PIPELINE);
    xAveraging.reset();
    yAveraging.reset();
    averagingTapsCurrent = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hasUseableTarget()) {
      Optional<Translation2d> cameraToTargetTranslationOptional = visionKinematics.forwardKinematics();
      if (cameraToTargetTranslationOptional.isPresent()) {
        // Use vision kinematics (perspective transform) to calculate the transform
        // between the camera and the target
        Transform2d cameraToTarget = new Transform2d(cameraToTargetTranslationOptional.get(), new Rotation2d());

        // Caputre the latest estimated pose from the odometry. We call this drifted
        // because we know the odometry algorithm has drifted from the actual field
        // frame
        Pose2d driftedFieldToVehiclePose = odometry.getCurrentPose();
        Transform2d driftedFieldToVehicle = new Transform2d(driftedFieldToVehiclePose.getTranslation(),
            driftedFieldToVehiclePose.getRotation());

        // Fuse odoetry measurement and camera measurement to get the field to target
        // transform *in our drifted coordinate system*
        Pose2d driftedFieldToTargetPose = driftedFieldToVehiclePose.transformBy(vehicleToCamera)
            .transformBy(cameraToTarget);
        Transform2d driftedFieldToTarget = new Transform2d(driftedFieldToTargetPose.getTranslation(), new Rotation2d());

        // Select the correct target
        Pose2d fieldToTarget;
        if (Math.abs(driftedFieldToVehicle.getRotation().getDegrees()) < 90 || alwaysUseFarTarget) {
          fieldToTarget = new Pose2d(Constants.fieldLength, Constants.visionTargetHorizDist * -1, new Rotation2d());
        } else {
          fieldToTarget = new Pose2d(0, Constants.visionTargetHorizDist, new Rotation2d());
        }

        // Transform our drifted coordinate system by known field values to obtain the
        // corrected field to vehicle
        Pose2d fieldToVehicle = fieldToTarget.transformBy(driftedFieldToTarget.inverse()) // field to drifted field
            .transformBy(driftedFieldToVehicle); // field to vehicle
        double averageX = xAveraging.calculate(fieldToVehicle.getTranslation().getX());
        double averageY = yAveraging.calculate(fieldToVehicle.getTranslation().getY());
        if (averagingTapsCurrent < averagingTapsTotal) {
          averagingTapsCurrent++;
        } else {
          double latency = (averagingTapsTotal * loopCycle * 0.5) + (limelight.getLatency() / 1000);
          odometry.setPosition(averageX, averageY, Timer.getFPGATimestamp() - latency);
        }
        return; // Exit before averaging data is reset
      }
    }

    // Reset if no target found
    xAveraging.reset();
    yAveraging.reset();
    averagingTapsCurrent = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLEDMode(LimelightLEDMode.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  /**
   * Returns whether the limelight is currently in a state where is is producing
   * targeting data that can be used for odometry.
   * 
   * @return Whether there is a useable target
   */
  private boolean hasUseableTarget() {
    return limelight.hasValidTarget() && limelight.getLEDMode() != LimelightLEDMode.OFF && !limelight.isDriverCam()
        && limelight.getCurrentPipeline() == odometryPipeline;
  }
}
