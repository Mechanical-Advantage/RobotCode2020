// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRoller;

public class RollerCharacterization extends CommandBase {
  private final ShooterRoller roller;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  ArrayList<Double> entries = new ArrayList<Double>();

  /** Creates a new Characterization. */
  public RollerCharacterization(ShooterRoller roller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roller);
    this.roller = roller;
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

    double autoSpeed = autoSpeedEntry.getDouble(0);
    entries.add(autoSpeed);

    entries.add(roller.getVoltage());
    entries.add(0.0);

    entries.add(roller.getPosition());
    entries.add(0.0);

    entries.add(roller.getVelocity());
    entries.add(0.0);

    entries.add(0.0);

    // Run at commanded speed
    roller.run((rotateEntry.getBoolean(false) ? -1 : 1) * autoSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.run(0);

    String data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    NetworkTableInstance.getDefault().flush();
    entries.clear();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
