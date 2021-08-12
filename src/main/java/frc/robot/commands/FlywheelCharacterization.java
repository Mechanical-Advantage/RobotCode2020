// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterFlyWheel;

public class FlywheelCharacterization extends CommandBase {
  private final ShooterFlyWheel flywheel;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  ArrayList<Double> entries = new ArrayList<Double>();

  double[] outputArray = new double[10];

  /** Creates a new Characterization. */
  public FlywheelCharacterization(ShooterFlyWheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    this.flywheel = flywheel;
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
    outputArray[0] = Timer.getFPGATimestamp();
    outputArray[1] = RobotController.getBatteryVoltage();

    outputArray[3] = flywheel.getVoltage();
    outputArray[4] = 0.0;

    outputArray[5] = flywheel.getPosition();
    outputArray[6] = 0.0;

    outputArray[7] = flywheel.getSpeed();
    outputArray[8] = 0.0;

    outputArray[9] = 0.0;

    // Run at commanded speed
    double autoSpeed = autoSpeedEntry.getDouble(0);
    flywheel.run((rotateEntry.getBoolean(false) ? -1 : 1) * autoSpeed);

    // Send full data set
    outputArray[2] = autoSpeed;
    for (double num : outputArray) {
      entries.add(num);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();

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
