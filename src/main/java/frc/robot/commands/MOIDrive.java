// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrainBase;

public class MOIDrive extends CommandBase {
  final DriveTrainBase driveTrain;
  final boolean doSpin;
  final AHRS ahrs;

  /** Creates a new MOIDriveForward. */
  public MOIDrive(DriveTrainBase driveTrain, AHRS ahrs, boolean doSpin) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.doSpin = doSpin;
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.drive(doSpin ? -0.5 : 0.5, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("MOI/LinearVelocityRadPerSec",
        (driveTrain.getRPSLeft() + driveTrain.getRPSRight()) * Math.PI);
    SmartDashboard.putNumber("MOI/AngularVelocityDegPerSec", ahrs.getRate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
