// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveTrainBase;

public class DriveCharacterization extends CommandBase {
  private final DriveTrainBase driveTrain;
  private final AHRS ahrs;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  ArrayList<Double> entries = new ArrayList<Double>();

  /** Creates a new Characterization. */
  public DriveCharacterization(DriveTrainBase driveTrain, AHRS ahrs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().setUpdateRate(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get telemetry data
    entries.add(Timer.getFPGATimestamp());
    entries.add(RobotController.getBatteryVoltage());

    entries.add(driveTrain.getVoltageLeft());
    entries.add(driveTrain.getVoltageRight());

    entries.add(driveTrain.getRotationsLeft() * (2 * (Math.PI)));
    entries.add(driveTrain.getRotationsRight() * (2 * (Math.PI)));

    entries.add(driveTrain.getRPSLeft() * (2 * (Math.PI)));
    entries.add(driveTrain.getRPSRight() * (2 * (Math.PI)));

    entries.add(ahrs.getAngle() * (Math.PI / 180));

    // Run at commanded speed
    double autoSpeed = autoSpeedEntry.getDouble(0);
    driveTrain.drive((rotateEntry.getBoolean(false) ? -1 : 1) * autoSpeed, autoSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();

    String data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
