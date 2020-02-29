/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.util.TunableNumber;

public class FeedUnstick extends CommandBase {

  private final ShooterRoller roller;
  private final Hopper hopper;
  private final TunableNumber hopperLeftSetpoint = new TunableNumber("Hopper/unstickSetpointLeft");
  private final TunableNumber hopperRightSetpoint = new TunableNumber("Hopper/unstickSetpointRight");
  private final TunableNumber rollerSetpoint = new TunableNumber("Shooter Roller/unstickSetpoint");
  private final UpdateLEDInterface updateLED;

  /**
   * Creates a new FeedUnstick.
   */
  public FeedUnstick(ShooterRoller roller, Hopper hopper, UpdateLEDInterface updateLED) {
    this.roller = roller;
    this.hopper = hopper;
    this.updateLED = updateLED;
    addRequirements(roller, hopper);
    hopperLeftSetpoint.setDefault(-0.5);
    hopperRightSetpoint.setDefault(-0.4);
    rollerSetpoint.setDefault(-0.6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateSetpoints();
    updateLED.update(OILED.SHOOTER_UNSTICK, OILEDState.ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      updateSetpoints();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.run(0, 0);
    roller.run(0);
    updateLED.update(OILED.SHOOTER_UNSTICK, OILEDState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateSetpoints() {
    roller.run(rollerSetpoint.get());
    hopper.run(hopperLeftSetpoint.get(), hopperRightSetpoint.get());
  }
}
