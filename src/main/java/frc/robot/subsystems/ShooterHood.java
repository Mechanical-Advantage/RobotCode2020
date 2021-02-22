/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.util.PressureSensor;

public class ShooterHood extends SubsystemBase {

  private static final int liftSolenoidChannel = 0;
  private static final int stopSolenoidChannel = 1;
  private static final double moveWait = 0.5; // Max secs to finishing raising or lowering lift
  private static final double minPressure = 40; // Min pressure to move

  private final PressureSensor pressureSensor;
  private final UpdateLEDInterface updateLED;
  private final SetHoodPositionLCDInterface setHoodLCD;
  private Solenoid liftSolenoid;
  private Solenoid stopSolenoid;
  private HoodPosition currentPosition = HoodPosition.UNKNOWN;
  private HoodPosition targetPosition = HoodPosition.UNKNOWN; // Unknown target disables movement
  private Timer moveTimer = new Timer();

  /**
   * Creates a new ShooterHood.
   */
  public ShooterHood(PressureSensor pressureSensor, UpdateLEDInterface updateLED,
      SetHoodPositionLCDInterface setHoodLCD) {
    if (Constants.getRobot() == RobotType.ROBOT_2020) {
      liftSolenoid = new Solenoid(liftSolenoidChannel);
      stopSolenoid = new Solenoid(stopSolenoidChannel);
    }
    this.pressureSensor = pressureSensor;
    this.updateLED = updateLED;
    this.setHoodLCD = setHoodLCD;
    moveTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (liftSolenoid == null || stopSolenoid == null) {
      return;
    }

    if (DriverStation.getInstance().isEnabled()) {
      if (currentPosition != targetPosition && targetPosition != HoodPosition.UNKNOWN && moveTimer.hasElapsed(moveWait)
          && pressureSensor.getPressure() > minPressure) { // We need to move and are able to
        switch (currentPosition) {
          case WALL:
            stopSolenoid.set(targetPosition == HoodPosition.LINE);
            liftSolenoid.set(true);
            currentPosition = targetPosition;
            break;
          case LINE:
            liftSolenoid.set(false);
            currentPosition = HoodPosition.WALL;
            break;
          case TRENCH:
          case UNKNOWN:
            stopSolenoid.set(false);
            liftSolenoid.set(false);
            currentPosition = HoodPosition.WALL;
            break;
        }
        moveTimer.reset();
      }
    } else {
      currentPosition = HoodPosition.UNKNOWN; // Always reset hood position when enabling
    }
    updateLED.update(OILED.HOOD_BOTTOM,
        targetPosition == HoodPosition.WALL ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
    updateLED.update(OILED.HOOD_MIDDLE,
        targetPosition == HoodPosition.LINE ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
    updateLED.update(OILED.HOOD_TOP,
        targetPosition == HoodPosition.TRENCH ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
  }

  /**
   * Starts moving the hood to a specified position
   * 
   * @param target The target position (UNKNOWN or null to disable movement)
   */
  public void setTargetPosition(HoodPosition target) {
    targetPosition = target == null ? HoodPosition.UNKNOWN : target;
    setHoodLCD.set(target);
  }

  /**
   * Retrieves the target position
   */
  public HoodPosition getTargetPosition() {
    return targetPosition;
  }

  /**
   * Checks whether the hood is at the target and not currently moving
   */
  public boolean atTargetPosition() {
    return (currentPosition == targetPosition || targetPosition == HoodPosition.UNKNOWN)
        && moveTimer.hasElapsed(moveWait);
  }

  public static enum HoodPosition {
    UNKNOWN, WALL, LINE, TRENCH
  }
}
