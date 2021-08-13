// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHood.HoodPosition;

public class RunShooterPreset extends CommandBase {

  private final ShooterFlyWheel flyWheel;
  private final ShooterHood hood;
  private final SendableChooser<ShooterPreset> chooser;

  /** Creates a new RunShooterPreset. */
  public RunShooterPreset(ShooterFlyWheel flyWheel, ShooterHood hood, SendableChooser<ShooterPreset> chooser) {
    addRequirements(flyWheel, hood);
    this.flyWheel = flyWheel;
    this.hood = hood;
    this.chooser = chooser;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flyWheel.setShooterRPM(chooser.getSelected().rpm);
    hood.setTargetPosition(chooser.getSelected().hoodPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class ShooterPreset {
    public final HoodPosition hoodPosition;
    public final double rpm;

    public ShooterPreset(HoodPosition hoodPosition, double rpm) {
      this.hoodPosition = hoodPosition;
      this.rpm = rpm;
    }
  }
}
