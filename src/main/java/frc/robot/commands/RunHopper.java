/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.util.TunableNumber;

public class RunHopper extends CommandBase {

  private TunableNumber setpointLeft = new TunableNumber("Hopper/setpointLeft");
  private TunableNumber setpointRight = new TunableNumber("Hopper/setpointRight");
  private final Hopper hopper;

  /**
   * Creates a new HopperCommand.
   * 
   * @param Hopper
   */
  public RunHopper(Hopper hopperSub) {
    hopper = hopperSub;
    addRequirements(hopperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpointLeft.setDefault(0.8);
    setpointRight.setDefault(0.7);
    hopper.run(setpointLeft.get(), setpointRight.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      hopper.run(setpointLeft.get(), setpointRight.get());
    }
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
