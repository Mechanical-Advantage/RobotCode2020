// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterFlyWheel;

public class WaitForShots extends CommandBase {

  private final ShooterFlyWheel flywheel;
  private final int requiredShots;
  private int startingShots;

  /**
   * Creates a new WaitForShots, which exits once the specified number of power
   * cells have been fired by the shooter. Does not require the flywheel subsytem.
   */
  public WaitForShots(ShooterFlyWheel flywheel, int requiredShots) {
    this.flywheel = flywheel;
    this.requiredShots = requiredShots;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingShots = flywheel.getShotCounter(); // Flywheel counter does not reset
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flywheel.getShotCounter() - startingShots >= requiredShots;
  }
}
