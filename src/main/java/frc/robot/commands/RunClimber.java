/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

  private static final double deadband = 0.05;

  private final Climber climber;
  private final DoubleSupplier stickAccess;
  private final DoubleSupplier horizStickAccess;

  /**
   * Creates a new RunClimber.
   */
  public RunClimber(Climber climber, DoubleSupplier stickAccess, DoubleSupplier horizStickAccess) {
    addRequirements(climber);
    this.climber = climber;
    this.stickAccess = stickAccess;
    this.horizStickAccess = horizStickAccess;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double stickVal = stickAccess.getAsDouble();
    stickVal = (Math.abs(stickVal) <= deadband) ? 0 : stickVal;
    double horizStickVal = horizStickAccess.getAsDouble();
    horizStickVal = (Math.abs(horizStickVal) <= deadband) ? 0 : horizStickVal;
    double left = stickVal, right = stickVal;
    if (horizStickVal > 0) {
      left *= (1 - Math.abs(horizStickVal));
    } else if (horizStickVal < 0) {
      right *= (1 - Math.abs(horizStickVal));
    }
    climber.run(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.run(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
