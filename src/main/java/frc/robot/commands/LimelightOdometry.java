/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.util.UtilFunctions;

public class LimelightOdometry extends CommandBase {

  private static final double targetHeight = 89.75; // in to center of target
  private static final double cameraHeight = 20;
  private static final double targetHorizLocation = Constants.visionTargetHorizDist; // in left of center line for close
                                                                                     // target
  // Can use the Limelight crosshair calibration instead of the next two options
  // (it's easier and compensates for the mount angle of the camera in the
  // Limelight)
  // To calibrate the Y value, put something at the same height as the camera and
  // adjust the crosshair Y until the crosshair lines up with the object
  // For our Limelight, set crosshair Y to -0.13 to make 0 degrees be directly in
  // front of the camera
  private static final double cameraVertAngle = 29; // 0 = straight forward, positive=up
  private static final double cameraHorizAngle = 0; // positive = right
  private static final double cameraHorizOffset = 0; // positive = right
  private static final int odometryPipeline = 1; // Data will be ignored for other pipelines

  private static final double heightDifference = targetHeight - cameraHeight; // How far above the camera the target is
  private LimelightInterface limelight;
  private RobotOdometry odometry;
  private boolean xCorrectionEnabled = true;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  @SuppressWarnings("all")
  public void execute() {
    if (hasUseableTarget()) {
      double poseAngle = odometry.getCurrentPose().getRotation().getDegrees() * -1;
      boolean farTarget = true;
      // Handle either target (always -90 to 90)
      if (poseAngle > 90) {
        poseAngle -= 180;
        farTarget = false;
      } else if (poseAngle < -90) {
        poseAngle += 180;
        farTarget = false;
      }
      poseAngle = UtilFunctions.boundHalfDegrees(poseAngle);

      double distance;
      if (xCorrectionEnabled) {
        distance = heightDifference
            / Math.tan(Math.toRadians(Math.abs(limelight.getTargetVertAngle() + cameraVertAngle)));
      } else {
        distance = odometry.getCurrentPose().getTranslation().getX();
        if (farTarget) {
          distance = Constants.fieldLength - distance;
        }
      }
      double horizAngle = limelight.getTargetHorizAngle() - cameraHorizAngle;
      if (cameraHorizOffset != 0) {
        // This is NOT the X dimension
        double horizDistance = Math.tan(Math.toRadians(horizAngle)) * distance;
        horizDistance += cameraHorizOffset;
        horizAngle = Math.toDegrees(Math.atan(horizDistance / distance));
      }
      // angle: In x y space, 0 = perpendicular to target
      double angle = poseAngle + horizAngle;
      // Sign of angle and xDist should be opposite
      double xDistance = Math.tan(Math.toRadians(angle)) * distance * -1;
      // Pass x/y to odometry as WPILib coordinate system
      double y = (xDistance + targetHorizLocation) * (farTarget ? -1 : 1);
      double timestamp = Timer.getFPGATimestamp() - (limelight.getLatency() / 1000);
      if (xCorrectionEnabled) {
        odometry.setPosition(farTarget ? Constants.fieldLength - distance : distance, y, timestamp);
      } else {
        odometry.setY(y, timestamp);
      }
    }
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
   * Set whether this command will change the x value in addition to the y value.
   * This is useful if the robot is at a known x value already. This can be
   * changed at any time.
   * 
   * @param enable Whether to enable x correction
   */
  public void enableXCorrection(boolean enable) {
    xCorrectionEnabled = enable;
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
