// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  double[] outputArray = new double[10];

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
    outputArray[0] = Timer.getFPGATimestamp();
    outputArray[1] = RobotController.getBatteryVoltage();

    outputArray[3] = driveTrain.getVoltageLeft();
    outputArray[4] = driveTrain.getVoltageRight();

    outputArray[5] = driveTrain.getRotationsLeft() * (2 * (Math.PI));
    outputArray[6] = driveTrain.getRotationsRight() * (2 * (Math.PI));

    outputArray[7] = driveTrain.getRPSLeft() * (2 * (Math.PI));
    outputArray[8] = driveTrain.getRPSRight() * (2 * (Math.PI));

    outputArray[9] = ahrs.getAngle() * (Math.PI / 180);

    // Run at commanded speed
    double autoSpeed = autoSpeedEntry.getDouble(0);
    driveTrain.drive((rotateEntry.getBoolean(false) ? -1 : 1) * autoSpeed, autoSpeed);

    // Send full data set
    outputArray[2] = autoSpeed;
    telemetryEntry.setDoubleArray(outputArray);
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
