/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.util.TunableNumber;

public class ShooterFlyWheelCommand extends CommandBase {

  private TunableNumber setPoint = new TunableNumber("Shooter FlyWheel/setpoint");
  private final ShooterFlyWheel shooterFlyWheel;

  /**
   * Creates a new ShooterFlyWheel.
   * 
   * @param ShooterFlyWheel
   */
  public ShooterFlyWheelCommand(ShooterFlyWheel shooterFlyWheelSub) {
    shooterFlyWheel = shooterFlyWheelSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterFlyWheelSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint.setDefault(0);
    shooterFlyWheel.run(setPoint.get());
    // shooterFlyWheel.setShooterPercentOutput(setPoint.get());
    // shooterFlyWheel.setShooterRPM(setPoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      shooterFlyWheel.run(setPoint.get());
      // shooterFlyWheel.setShooterPercentOutput(setPoint.get());
      // shooterFlyWheel.setShooterRPM(setPoint.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
