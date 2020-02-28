/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;

/**
 * Controls the limelight when nothing else is using it
 */
public class IdleLimelightControl extends CommandBase {

  private static final int pipeline = 1;
  // Whether to blink the limelight when there is no target to try to find one
  private static final boolean blink = true;
  // How often to turn on the limelight to try to get data
  private static final double blinkWaitTimeNoTarget = 10;
  private static final double onTimeNoTarget = 1;
  private static final double blinkWaitTimeTarget = 3;
  private static final double onTimeTarget = 1;
  // Whether to leave the limelight on all the time when a target is visible
  private static final boolean continuousTracking = false;
  // How long to wait after the target is lost before considering it out of range
  private static final double targetLossDelay = 2;

  private final LimelightInterface limelight;
  private boolean hasTarget;
  private boolean ledsOn;
  private final Timer targetLossTimer = new Timer();
  private final Timer blinkTimer = new Timer();

  /**
   * Creates a new IdleLimelightControl.
   */
  public IdleLimelightControl(LimelightInterface limelight) {
    addRequirements(limelight);
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(pipeline);
    targetLossTimer.start();
    targetLossTimer.reset();
    blinkTimer.start();
    blinkTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ledsOn && limelight.hasValidTarget()) {
      targetLossTimer.reset();
      hasTarget = true;
    } else if (ledsOn && targetLossTimer.hasElapsed(targetLossDelay)) {
      hasTarget = false;
    }
    if (blink && DriverStation.getInstance().isEnabled()) {
      double currentStageTime;
      if (ledsOn) {
        if (hasTarget) {
          currentStageTime = onTimeTarget;
        } else {
          currentStageTime = onTimeNoTarget;
        }
      } else {
        if (hasTarget) {
          currentStageTime = blinkWaitTimeTarget;
        } else {
          currentStageTime = blinkWaitTimeNoTarget;
        }
      }

      if (!(continuousTracking && ledsOn && hasTarget) && blinkTimer.advanceIfElapsed(currentStageTime)) {
        if (ledsOn) {
          ledsOn = false;
          limelight.setLEDMode(LimelightLEDMode.OFF);
        } else {
          ledsOn = true;
          limelight.setLEDMode(LimelightLEDMode.ON);
        }
      }
    } else {
      limelight.setLEDMode(LimelightLEDMode.OFF);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetLossTimer.stop();
    blinkTimer.stop();
    limelight.setLEDMode(LimelightLEDMode.PIPELINE);
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
}
