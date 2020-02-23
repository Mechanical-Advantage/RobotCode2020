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
import frc.robot.subsystems.Intake;
import frc.robot.util.UpdateLEDInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeExtendRetract extends InstantCommand {

  private final boolean extend;
  private final Intake intake;
  private final UpdateLEDInterface updateLED;

  public IntakeExtendRetract(boolean extend, Intake intake, UpdateLEDInterface updateLED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extend = extend;
    this.intake = intake;
    this.updateLED = updateLED;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (extend) {
      intake.extend();
    } else {
      intake.retract();
    }
    updateLED.update(OILED.INTAKE_EXTEND, extend ? OILEDState.ON : OILEDState.OFF);
    updateLED.update(OILED.INTAKE_RETRACT, extend ? OILEDState.OFF : OILEDState.ON);
  }
}
