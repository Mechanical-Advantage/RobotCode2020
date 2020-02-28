/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.util.SetHoodPositionLCDInterface;
import frc.robot.util.UpdateLEDInterface;

public class ShooterHood extends SubsystemBase {

  private static final int liftSolenoidChannel = 0;
  private static final int stopSolenoidChannel = 1;

  private Solenoid liftSolenoid;
  private Solenoid stopSolenoid;

  private final UpdateLEDInterface updateLED;
  private final SetHoodPositionLCDInterface setHoodLCD;

  /**
   * Creates a new ShooterHood.
   */
  public ShooterHood(UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    this.updateLED = updateLED;
    this.setHoodLCD = setHoodLCD;
    if (!available()) {
      return;
    }
    liftSolenoid = new Solenoid(liftSolenoidChannel);
    stopSolenoid = new Solenoid(stopSolenoidChannel);
  }

  private boolean available() {
    return Constants.getRobot() == RobotType.ROBOT_2020;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the position of the lift solenoid
   * 
   * @param raised
   */
  public void setLiftPosition(boolean raised) {
    if (available()) {
      liftSolenoid.set(raised);
    }
  }

  /**
   * Sets whether the stop solenoid is extended
   * 
   * @param stopped
   */
  public void setStopPosition(boolean stopped) {
    if (available()) {
      if (!(liftSolenoid.get() && stopped)) { // do not lock if lift raised
        stopSolenoid.set(stopped);
      }
    }
  }

  /**
   * Updates hood position LEDs
   * 
   * @param led LED to illuminate
   */
  public void setLEDs(HoodPosition position) {
    setHoodLCD.set(position);
    updateLED.update(OILED.HOOD_BOTTOM, position == HoodPosition.BOTTOM ? OILEDState.ON : OILEDState.OFF);
    updateLED.update(OILED.HOOD_MIDDLE, position == HoodPosition.MIDDLE ? OILEDState.ON : OILEDState.OFF);
    updateLED.update(OILED.HOOD_TOP, position == HoodPosition.TOP ? OILEDState.ON : OILEDState.OFF);
  }

  public static enum HoodPosition {
    BOTTOM, MIDDLE, TOP
  }
}
