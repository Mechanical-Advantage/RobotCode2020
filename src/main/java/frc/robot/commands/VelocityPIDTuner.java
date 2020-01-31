/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.util.TunableNumber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveTrainBase;

/**
 * An auto command for tuning velocity PIDs on the drive train
 */
public class VelocityPIDTuner extends CommandBase {

  private static final boolean spin = false;
  private DriveTrainBase driveSubsystem;

  private TunableNumber P = new TunableNumber("Drive PID/P");
  private TunableNumber I = new TunableNumber("Drive PID/I");
  private TunableNumber D = new TunableNumber("Drive PID/D");
  private TunableNumber F = new TunableNumber("Drive PID/F");
  private TunableNumber setpoint = new TunableNumber("Drive PID/setpoint");

  public VelocityPIDTuner(DriveTrainBase drive) {
    addRequirements(drive);
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    P.setDefault(driveSubsystem.getP());
    I.setDefault(driveSubsystem.getI());
    D.setDefault(driveSubsystem.getD());
    F.setDefault(driveSubsystem.getF());
    setpoint.setDefault(0);
    SmartDashboard.putBoolean("Drive PID/enabled", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setPID(P.get(), I.get(), D.get(), F.get(), 0);
    if (SmartDashboard.getBoolean("Drive PID/enabled", false)) {
      driveSubsystem.driveInchesPerSec(setpoint.get(), setpoint.get() * (spin ? -1 : 1));
      SmartDashboard.putNumber("Drive velocity",
          (driveSubsystem.getVelocityLeft() + driveSubsystem.getVelocityRight()) / 2);
    } else {
      driveSubsystem.neutralOutput();
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
