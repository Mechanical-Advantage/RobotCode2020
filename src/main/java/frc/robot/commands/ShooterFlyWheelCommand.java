/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.util.TunableNumber;

public class ShooterFlyWheelCommand extends CommandBase {

  private TunableNumber setpoint = new TunableNumber("Shooter FlyWheel/setpoint");
  private final ShooterFlyWheel shooterFlyWheel;

  /**
   * Creates a new ShooterFlyWheel.
   * 
   * @param ShooterFlyWheel
   */
  public ShooterFlyWheelCommand(ShooterFlyWheel shooterFlyWheelSub) {
    shooterFlyWheel = shooterFlyWheelSub;
    addRequirements(shooterFlyWheelSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint.setDefault(0);
    // shooterFlyWheel.run(setpoint.get());
    // shooterFlyWheel.setShooterPercentOutput(setpoint.get());
    shooterFlyWheel.setShooterRPM(setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      // shooterFlyWheel.run(setpoint.get());
      // shooterFlyWheel.setShooterPercentOutput(setpoint.get());
      shooterFlyWheel.setShooterRPM(setpoint.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterFlyWheel.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
