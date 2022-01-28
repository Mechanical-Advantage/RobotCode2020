// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.TunableNumber;

public class HoldPosition extends CommandBase {

  private final DriveTrainBase driveTrain;

  private TunableNumber kP = new TunableNumber("HoldPosition/kP");
  private TunableNumber kI = new TunableNumber("HoldPosition/kI");
  private TunableNumber kD = new TunableNumber("HoldPosition/kD");

  private final PIDController leftController;
  private final PIDController rightController;

  /** Creates a new HoldPosition. */
  public HoldPosition(DriveTrainBase driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    switch (Constants.getRobot()) {
      case ROBOT_2020:
        kP.setDefault(1);
        kI.setDefault(0);
        kD.setDefault(0);
        break;
      default:
        kP.setDefault(0);
        kI.setDefault(0);
        kD.setDefault(0);
    }
    leftController = new PIDController(kP.get(), kI.get(), kD.get());
    rightController = new PIDController(kP.get(), kI.get(), kD.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftController.reset();
    rightController.reset();
    leftController.setSetpoint(driveTrain.getDistanceLeft());
    rightController.setSetpoint(driveTrain.getDistanceRight());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      leftController.setPID(kP.get(), kI.get(), kD.get());
      rightController.setPID(kP.get(), kI.get(), kD.get());
    }

    double leftSpeed = leftController.calculate(driveTrain.getDistanceLeft());
    double rightSpeed = rightController.calculate(driveTrain.getDistanceRight());
    driveTrain.driveInchesPerSec(leftSpeed, rightSpeed);
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
