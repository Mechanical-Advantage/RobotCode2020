/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.subsystems.ShooterHood.HoodPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetHoodPositionLEDs extends InstantCommand {
  private final HoodPosition position;
  private final UpdateLEDInterface updateLED;
  private final SetHoodPositionLCDInterface setHoodLCD;
  private final OILEDState state;

  public SetHoodPositionLEDs(HoodPosition position, UpdateLEDInterface updateLED,
      SetHoodPositionLCDInterface setHoodLCD, OILEDState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.position = position;
    this.updateLED = updateLED;
    this.setHoodLCD = setHoodLCD;
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setHoodLCD.set(position);
    updateLED.update(OILED.HOOD_BOTTOM, position == HoodPosition.WALL ? state : OILEDState.OFF);
    updateLED.update(OILED.HOOD_MIDDLE, position == HoodPosition.LINE ? state : OILEDState.OFF);
    updateLED.update(OILED.HOOD_TOP, position == HoodPosition.TRENCH ? state : OILEDState.OFF);
  }
}
