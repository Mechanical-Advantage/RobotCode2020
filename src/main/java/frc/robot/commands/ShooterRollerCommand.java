/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.util.TunableNumber;

public class ShooterRollerCommand extends CommandBase {

  private TunableNumber setpoint = new TunableNumber("Shooter Roller PID/setpoint");
  private final ShooterRoller shooterRoller = new ShooterRoller();
  private double rpm;
  private boolean tuningMode = false;

  /**
   * Creates a new ShooterRoller.
   * 
   * @param ShooterRoller
   */
  public ShooterRollerCommand(Subsystem ShooterRoller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint.setDefault(0);
    shooterRoller.setShooterRPM(setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tuningMode) {
      setpoint.get();
      // update setpoint
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
