/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.util.TunableNumber;
import frc.robot.util.UpdateLEDInterface;

public class RunShooterFlyWheel extends CommandBase {

  private static final double percentThreshold = 5; // what percent of target rpm should we be at

  private TunableNumber setpoint = new TunableNumber("Shooter FlyWheel/setpoint");
  private final ShooterFlyWheel shooterFlyWheel;
  private final UpdateLEDInterface updateLED;
  private boolean lastUpToSpeed = false;
  private boolean shooterLEDInitialized = false;

  /**
   * Creates a new ShooterFlyWheel.
   * 
   * @param ShooterFlyWheel
   */
  public RunShooterFlyWheel(ShooterFlyWheel shooterFlyWheelSub, UpdateLEDInterface updateLED) {
    shooterFlyWheel = shooterFlyWheelSub;
    this.updateLED = updateLED;
    addRequirements(shooterFlyWheelSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint.setDefault(6000);
    // shooterFlyWheel.run(setpoint.get());
    shooterFlyWheel.setShooterRPM(setpoint.get());
    setRunningLEDs(true);
    shooterLEDInitialized = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      // shooterFlyWheel.run(setpoint.get());
      shooterFlyWheel.setShooterRPM(setpoint.get());
    }
    double targetRpm = shooterFlyWheel.getShooterSetpoint();
    double threshold = targetRpm * (1 - (percentThreshold / 100));
    boolean upToSpeed = shooterFlyWheel.getSpeed() > threshold;
    if (upToSpeed != lastUpToSpeed || !shooterLEDInitialized) {
      shooterLEDInitialized = true;
      upToSpeed = lastUpToSpeed;
      updateLED.update(OILED.SHOOTER_SHOOT, upToSpeed ? OILEDState.ON : OILEDState.PULSE_FAST);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterFlyWheel.stop();
    setRunningLEDs(false);
    updateLED.update(OILED.SHOOTER_SHOOT, OILEDState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void setRunningLEDs(boolean running) {
    updateLED.update(OILED.SHOOTER_RUN, running ? OILEDState.ON : OILEDState.OFF);
    updateLED.update(OILED.SHOOTER_STOP, running ? OILEDState.OFF : OILEDState.ON);
  }
}
