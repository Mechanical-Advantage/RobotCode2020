/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.util.UpdateLEDInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLEDOverride extends InstantCommand {
  private final OILED led;
  private final OILEDState state;
  private final UpdateLEDInterface updateLED;

  public SetLEDOverride(OILED led, OILEDState state, UpdateLEDInterface updateLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.state = state;
    this.updateLED = updateLED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateLED.update(led, state);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
