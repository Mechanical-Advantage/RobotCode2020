/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.util.TunableNumber;

public class RunHopper extends CommandBase {

  private TunableNumber minSpeed = new TunableNumber("Hopper/feedMinSpeed");
  private TunableNumber maxSpeed = new TunableNumber("Hopper/feedMaxSpeed");
  private TunableNumber oscillationRate = new TunableNumber("Hopper/feedOscillationRate");
  private final Hopper hopper;

  /**
   * Creates a new HopperCommand.
   * 
   * @param Hopper
   */
  public RunHopper(Hopper hopperSub) {
    hopper = hopperSub;
    addRequirements(hopperSub);
    minSpeed.setDefault(0.5);
    maxSpeed.setDefault(1);
    oscillationRate.setDefault(8);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = Math.sin((Timer.getFPGATimestamp() * oscillationRate.get()) / 3);
    double rightSpeed = Math.sin(((Timer.getFPGATimestamp() + Math.PI) * oscillationRate.get()) / 7);
    leftSpeed = minSpeed.get() + (0.5 * (maxSpeed.get() - minSpeed.get()))
        + (leftSpeed * 0.5 * (maxSpeed.get() - minSpeed.get()));
    rightSpeed = minSpeed.get() + (0.5 * (maxSpeed.get() - minSpeed.get()))
        + (rightSpeed * 0.5 * (maxSpeed.get() - minSpeed.get()));
    hopper.run(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.run(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
