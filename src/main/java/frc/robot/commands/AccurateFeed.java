// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;

public class AccurateFeed extends CommandBase {

  private static final double shootRollerSpeed = 1;
  private static final double shootHopperLeftSpeed = 0.8;
  private static final double shootHopperRightSpeed = 0.7;
  private static final double ejectRollerSpeed = -1;
  private static final double ejectHopperLeftSpeed = 0;
  private static final double ejectHopperRightSpeed = 0;

  private final ShooterRoller roller;
  private final Hopper hopper;
  private final ShooterFlyWheel flywheel;
  private final ShooterHood hood;

  /** Creates a new AccurateFeed. */
  public AccurateFeed(ShooterRoller roller, Hopper hopper, ShooterFlyWheel flywheel, ShooterHood hood) {
    addRequirements(roller, hopper);
    this.roller = roller;
    this.hopper = hopper;
    this.flywheel = flywheel;
    this.hood = hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hood.atTargetPosition() && flywheel.readyForAccurateFeed()) {
      roller.run(shootRollerSpeed);
      hopper.run(shootHopperLeftSpeed, shootHopperRightSpeed);
    } else {
      roller.run(ejectRollerSpeed);
      hopper.run(ejectHopperLeftSpeed, ejectHopperRightSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.run(0, 0);
    roller.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
