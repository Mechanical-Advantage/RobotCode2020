/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.util.TunableNumber;

public class RunShooterRoller extends CommandBase {

  private TunableNumber setpoint = new TunableNumber("Roller/Setpoint");
  private final ShooterRoller shooterRoller;

  /**
   * Creates a new ShooterRoller.
   * 
   * @param ShooterRoller
   */
  public RunShooterRoller(ShooterRoller shooterRollerSub) {
    shooterRoller = shooterRollerSub;
    addRequirements(shooterRollerSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint.setDefault(5000);
    shooterRoller.runClosedLoop(setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      shooterRoller.runClosedLoop(setpoint.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRoller.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
