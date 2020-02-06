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
import frc.robot.util.UtilFunctions;

public class LimelightOdometry extends CommandBase {

  private static final double targetHeight = 67.75; // in to center of target
  private static final double cameraHeight = 25.75;
  private static final double targetHorizLocation = 0; // in left of center line for close target
  // Can use the Limelight crosshair calibration instead of the next two options
  // (it's easier and compensates for the mount angle of the camera in the
  // Limelight)
  // To calibrate the Y value, put something at the same height as the camera and
  // adjust the crosshair Y until the crosshair lines up with the object
  // For our Limelight, set crosshair Y to -0.13 to make 0 degrees be directly in
  // front of the camera
  private static final double cameraVertAngle = 0; // 0 = straight forward, positive=up
  private static final double cameraHorizAngle = 0; // positive = right
  private static final double cameraHorizOffset = 0; // positive = right

  private static final double heightDifference = targetHeight - cameraHeight; // How far above the camera the target is
  private LimelightInterface limelight;
  private RobotOdometry odometry;

  /**
   * Creates a new LimelightOdometry.
   */
  public LimelightOdometry(LimelightInterface limelight, RobotOdometry odometry) {
    addRequirements(limelight, odometry);
    this.limelight = limelight;
    this.odometry = odometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  @SuppressWarnings("all")
  public void execute() {
    if (limelight.hasValidTarget()) {
      double distance = heightDifference
          / Math.tan(Math.toRadians(Math.abs(limelight.getTargetVertAngle() + cameraVertAngle)));
      double horizAngle = limelight.getTargetHorizAngle() - cameraHorizAngle;
      if (cameraHorizOffset != 0) {
        // This is NOT the X dimension
        double horizDistance = Math.tan(Math.toRadians(horizAngle)) * distance;
        horizDistance += cameraHorizOffset;
        horizAngle = Math.toDegrees(Math.atan(horizDistance / distance));
      }
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
      // angle: In x y space, 0 = perpendicular to target
      double angle = poseAngle + horizAngle;
      // Sign of angle and xDist should be opposite
      double xDistance = Math.tan(Math.toRadians(angle)) * distance * -1;
      // Pass x/y to odometry as WPILib coordinate system
      odometry.setPosition(farTarget ? Constants.fieldLength - distance : distance,
          (xDistance + targetHorizLocation) * (farTarget ? -1 : 1), Timer.getFPGATimestamp() - limelight.getLatency());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
