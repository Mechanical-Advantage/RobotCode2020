/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI class to support the old revenge operator panel powered by two eStop
 * robotics boards.
 */
public class OIeStopConsole implements IDriverOverrideOI, IOperatorOI {
  private Joystick oiController1;
  private Joystick oiController2;

  private Button openLoopDrive;
  private Button driveDisableSwitch;
  private Button shiftDisableSwitch;

  private Button shooterFlywheelRunButton;
  private Button shooterFlywheelStopButton;
  private Button shooterRollerButton;

  private Button intakeExtendButton;
  private Button intakeRetractButton;
  private Button intakeForwardsButton;
  private Button intakeBackwardsButton;

  NetworkTable ledTable;
  NetworkTableEntry ledEntry;

  private static final Map<OILED, Integer> ledMap = new HashMap<OILED, Integer>();

  public OIeStopConsole(int firstID, int secondID) {
    oiController1 = new Joystick(firstID);
    oiController2 = new Joystick(secondID);

    openLoopDrive = new JoystickButton(oiController2, 10);
    driveDisableSwitch = new JoystickButton(oiController2, 9);
    shiftDisableSwitch = new JoystickButton(oiController2, 8);

    shooterFlywheelRunButton = new JoystickButton(oiController2, 4);
    shooterFlywheelStopButton = new JoystickButton(oiController2, 3);
    shooterRollerButton = new JoystickButton(oiController2, 5);

    intakeExtendButton = new JoystickButton(oiController1, 9);
    intakeRetractButton = new JoystickButton(oiController1, 10);
    intakeForwardsButton = new JoystickButton(oiController2, 2);
    intakeBackwardsButton = new JoystickButton(oiController2, 1);

    ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
    ledEntry = ledTable.getEntry("OI LEDs");

    // Define LED mapping
    ledMap.put(OILED.MISC_1, 0);
    ledMap.put(OILED.MISC_2, 1);
    ledMap.put(OILED.MISC_3, 2);

    ledEntry.setBooleanArray(new boolean[] { false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false });
  }

  @Override
  public Trigger getOpenLoopSwitch() {
    return openLoopDrive;
  }

  @Override
  public Trigger getDriveDisableSwitch() {
    return driveDisableSwitch;
  }

  @Override
  public Trigger getShiftLockSwitch() {
    return shiftDisableSwitch;
  }

  @Override
  public Trigger getShooterFlywheelRunButton() {
    return shooterFlywheelRunButton;
  }

  @Override
  public Trigger getShooterFlywheelStopButton() {
    return shooterFlywheelStopButton;
  }

  @Override
  public Trigger getShooterRollerButton() {
    return shooterRollerButton;
  }

  @Override
  public Trigger getIntakeExtendButton() {
    return intakeExtendButton;
  }

  @Override
  public Trigger getIntakeRetractButton() {
    return intakeRetractButton;
  }

  @Override
  public Trigger getRunIntakeForwardsButton() {
    return intakeForwardsButton;
  }

  @Override
  public Trigger getRunIntakeBackwardsButton() {
    return intakeBackwardsButton;
  }

  @Override
  public void updateLED(OILED led, OILEDState state) {
    if (ledMap.containsKey(led)) {
      boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[] { false, false, false, false, false,
          false, false, false, false, false, false, false, false, false, false, false, false });
      array[ledMap.get(led)] = state != OILEDState.OFF;
      ledEntry.setBooleanArray(array);
    }
  }
}
