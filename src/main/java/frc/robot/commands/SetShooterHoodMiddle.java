/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.subsystems.ShooterHood;
import frc.robot.util.PressureSensor;
import frc.robot.util.UpdateLEDInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShooterHoodMiddle extends SequentialCommandGroup {

  private static final double raiseWait = 0.25; // seconds to wait after lowering before moving back up

  /**
   * Creates a new SetShooterHoodMiddle.
   */
  public SetShooterHoodMiddle(ShooterHood shooterHood, PressureSensor pressureSensor, UpdateLEDInterface updateLED) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new WaitForPressure(pressureSensor),
        new InstantCommand(() -> updateLED.update(OILED.HOOD_TOP, OILEDState.OFF)),
        new InstantCommand(() -> updateLED.update(OILED.HOOD_MIDDLE, OILEDState.ON)),
        new InstantCommand(() -> updateLED.update(OILED.HOOD_BOTTOM, OILEDState.OFF)),
        new InstantCommand(() -> shooterHood.setStopPosition(false), shooterHood),
        new InstantCommand(() -> shooterHood.setLiftPosition(false), shooterHood), new WaitCommand(raiseWait),
        new InstantCommand(() -> shooterHood.setStopPosition(true), shooterHood),
        new InstantCommand(() -> shooterHood.setLiftPosition(true), shooterHood));
  }
}
