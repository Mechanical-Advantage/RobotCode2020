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
import frc.robot.subsystems.ShooterHood;
import frc.robot.util.PressureSensor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShooterHoodMiddleTop extends SequentialCommandGroup {

  private static final double raiseWait = 0.5; // seconds to wait after lowering before moving back up

  /**
   * Creates a new SetShooterHoodMiddle.
   */
  public SetShooterHoodMiddleTop(ShooterHood shooterHood, PressureSensor pressureSensor, boolean top) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new WaitForPressure(pressureSensor),
        new InstantCommand(() -> shooterHood.setStopPosition(false), shooterHood),
        new InstantCommand(() -> shooterHood.setLiftPosition(false), shooterHood), new WaitCommand(raiseWait),
        new InstantCommand(() -> shooterHood.setStopPosition(!top), shooterHood),
        new InstantCommand(() -> shooterHood.setLiftPosition(true), shooterHood));
  }
}
