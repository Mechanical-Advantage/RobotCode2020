/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

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
  private Joystick oiController1 = new Joystick(2);
  private Joystick oiController2 = new Joystick(3);

  private Button openLoopDrive = new JoystickButton(oiController2, 10);
  private Button driveDisableSwitch = new JoystickButton(oiController2, 9);
  private Button shiftDisableSwitch = new JoystickButton(oiController2, 8);
  private Button shooterFlywheelRunButton = new JoystickButton(oiController2, 4);
  private Button shooterFlywheelStopButton = new JoystickButton(oiController2, 3);
  private Button shooterRollerButton = new JoystickButton(oiController2, 5);

  private Button intakeExtendButton = new JoystickButton(oiController1, 9);
  private Button intakeRetractButton = new JoystickButton(oiController1, 10);
  private Button intakeForwardsButton = new JoystickButton(oiController2, 2);
  private Button intakeBackwardsButton = new JoystickButton(oiController2, 1);

  NetworkTable ledTable;
  NetworkTableEntry ledEntry;

  public OIeStopConsole() {
    ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
    ledEntry = ledTable.getEntry("OI LEDs");

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
  public void updateLED(OILED led, boolean state) {
    boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[] { false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false, false, false });
    array[led.ordinal()] = state;
    ledEntry.setBooleanArray(array);
  }
}
