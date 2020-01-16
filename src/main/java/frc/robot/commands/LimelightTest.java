/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightInterface;

public class LimelightTest extends CommandBase {

  private static final double targetHeight = 67.75; // in to center of target
  private static final double cameraHeight = 25.75;
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
  private final LimelightInterface limelight;
  private final AHRS ahrs;
  private double initialAngle;

  /**
   * Creates a new LimelightTest.
   */
  public LimelightTest(LimelightInterface limelight, AHRS ahrs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    this.limelight = limelight;
    this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This command assumes it is started perpendicular to the target
    initialAngle = ahrs.getAngle();
    System.out.println("Initial Angle : " + initialAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  // Using a final boolean for comparisons creates dead code warnings and one of
  // them can't be suppressed specifically so this needs to suppress everything
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
      // angle: In x y space, 0 = perpendicular to target
      double angle = (ahrs.getAngle() - initialAngle) + horizAngle;
      // Sign of angle and xDist should be opposite
      double xDistance = Math.tan(Math.toRadians(angle)) * distance * -1;
      System.out.println(
          "Dist: " + distance + " X Dist: " + xDistance + " Horiz Angle: " + horizAngle + " Cartesian Angle: " + angle);
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
