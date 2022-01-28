/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;

public class ShooterHood extends SubsystemBase {

  private static final int liftSolenoidChannel = 0;
  private static final int stopSolenoidChannel = 1;
  private static final double liftMoveWait = 0.3; // Max secs to finishing raising or lowering lift
  private static final double stopMoveWait = 0.15; // Max secs for stops to move when in trench
  private static final double minPressure = 20; // Min pressure to move

  private final Pneumatics pneumatics;
  private final UpdateLEDInterface updateLED;
  private final SetHoodPositionLCDInterface setHoodLCD;
  private Solenoid liftSolenoid;
  private Solenoid stopSolenoid;
  private HoodPosition currentPosition = HoodPosition.UNKNOWN;
  private HoodPosition targetPosition = HoodPosition.UNKNOWN; // Forces reset to known position
  private Timer moveTimer = new Timer();
  private double currentMoveWait = liftMoveWait;

  /**
   * Creates a new ShooterHood.
   */
  public ShooterHood(Pneumatics pneumatics, UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    if (Constants.getRobot() == RobotType.ROBOT_2020) {
      liftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, liftSolenoidChannel);
      stopSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, stopSolenoidChannel);
    }
    this.pneumatics = pneumatics;
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

    if (DriverStation.isEnabled()) {
      if (currentPosition != targetPosition && targetPosition != HoodPosition.UNKNOWN
          && moveTimer.hasElapsed(currentMoveWait)
          && (pneumatics.getPressure() > minPressure || !pneumatics.isSensorConnected())) { // We need to move and are
                                                                                            // able to. Override
                                                                                            // pressure check if sensor
                                                                                            // disconnects.
        currentMoveWait = liftMoveWait; // Wait for lift by default
        switch (currentPosition) {
          case FRONT_LINE:
            stopSolenoid.set(false);
            liftSolenoid.set(false);
            currentPosition = HoodPosition.WALL;
            break;
          case WALL:
            stopSolenoid.set(targetPosition == HoodPosition.FRONT_LINE);
            liftSolenoid.set(true);
            currentPosition = targetPosition == HoodPosition.FRONT_LINE ? HoodPosition.FRONT_LINE : HoodPosition.TRENCH;
            break;
          case TRENCH:
            if (stopSolenoid.get() != (targetPosition == HoodPosition.BACK_LINE)) {
              stopSolenoid.set(targetPosition == HoodPosition.BACK_LINE);
              currentMoveWait = stopMoveWait; // We don't need to wait as long for the stops
            } else {
              liftSolenoid.set(false);
              currentPosition = targetPosition == HoodPosition.BACK_LINE ? HoodPosition.BACK_LINE : HoodPosition.WALL;
            }
            break;
          case BACK_LINE:
            liftSolenoid.set(true);
            stopSolenoid.set(targetPosition == HoodPosition.TRENCH);
            currentPosition = HoodPosition.TRENCH;
            break;
          case UNKNOWN:
            liftSolenoid.set(true);
            stopSolenoid.set(false);
            currentPosition = HoodPosition.TRENCH;
            break;
        }
        moveTimer.reset();
      }
    } else {
      if (currentPosition != HoodPosition.UNKNOWN) {
        if (currentPosition == HoodPosition.BACK_LINE) {
          currentPosition = HoodPosition.UNKNOWN;
        } else {
          currentPosition = HoodPosition.WALL;
        }
      }
    }

    updateLED.update(OILED.HOOD_WALL,
        targetPosition == HoodPosition.WALL ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
    updateLED.update(OILED.HOOD_FRONT_LINE,
        targetPosition == HoodPosition.FRONT_LINE ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
    updateLED.update(OILED.HOOD_BACK_LINE,
        targetPosition == HoodPosition.BACK_LINE ? (atTargetPosition() ? OILEDState.ON : OILEDState.PULSE_FAST)
            : OILEDState.OFF);
    updateLED.update(OILED.HOOD_TRENCH,
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
        && moveTimer.hasElapsed(currentMoveWait);
  }

  public static enum HoodPosition {
    UNKNOWN, WALL, FRONT_LINE, BACK_LINE, TRENCH
  }
}
