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

  private static final double targetHeight = 67; // in to center of target
  private static final double cameraHeight = 25.5;
  private static final double cameraVertAngle = 4; // 0 = straight forward, positive=up
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
    initialAngle = ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.hasValidTarget()) {
      double distance = heightDifference
          / Math.tan(Math.toRadians(Math.abs(limelight.getTargetVertAngle() + cameraVertAngle)));
      double horizAngle = limelight.getTargetHorizAngle() - cameraHorizAngle;
      double horizDistance = Math.tan(Math.toRadians(horizAngle)) * distance;
      if (cameraHorizOffset != 0) {
        horizDistance += cameraHorizOffset;
        horizAngle = Math.toDegrees(Math.atan(horizDistance / distance));
      }
      System.out.println("Dist: " + distance + " Horiz Dist: " + horizDistance + " Horiz Angle: " + horizAngle);
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
