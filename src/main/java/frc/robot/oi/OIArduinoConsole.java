/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
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
 * OI class for the Arduino Leonardo based box/panel.
 */
public class OIArduinoConsole implements IOperatorOI, IDriverOverrideOI {

    private Joystick arduinoController1;
    private Joystick arduinoController2;

    private Button openLoopSwitch;
    private Button driveDisableSwitch;
    private Button manualHoodSwitch;
    private Button buddyClimbSwitch;
    private Button climbEnableSwitch;

    private Button intakeExtendButton;
    private Button intakeRetractButton;
    private Button intakeForwardsButton;
    private Button intakeBackwardsButton;

    private Button shooterFlywheelRunButton;
    private Button shooterFlywheelStopButton;
    private Button shooterRollerButton;
    private Button shooterUnstickButton;

    private Button hoodWallButton;
    private Button hoodLineButton;
    private Button hoodTrenchButton;

    NetworkTable ledTable;
    NetworkTableEntry ledEntry;

    private static final Map<OILED, Integer> ledMap = new HashMap<OILED, Integer>();

    public OIArduinoConsole(int firstID, int secondID) {
        arduinoController1 = new Joystick(firstID);
        arduinoController2 = new Joystick(secondID);

        openLoopSwitch = new JoystickButton(arduinoController1, 1);
        driveDisableSwitch = new JoystickButton(arduinoController1, 2);
        manualHoodSwitch = new JoystickButton(arduinoController1, 4);
        buddyClimbSwitch = new JoystickButton(arduinoController1, 5);
        climbEnableSwitch = new JoystickButton(arduinoController1, 6);

        intakeExtendButton = new JoystickButton(arduinoController1, 7);
        intakeRetractButton = new JoystickButton(arduinoController1, 8);
        intakeForwardsButton = new JoystickButton(arduinoController1, 9);
        intakeBackwardsButton = new JoystickButton(arduinoController1, 10);

        shooterFlywheelRunButton = new JoystickButton(arduinoController2, 2); // 14
        shooterFlywheelStopButton = new JoystickButton(arduinoController2, 1); // 13
        shooterRollerButton = new JoystickButton(arduinoController2, 5); // 17
        shooterUnstickButton = new JoystickButton(arduinoController2, 3); // 15

        hoodWallButton = new JoystickButton(arduinoController2, 6); // 18
        hoodLineButton = new JoystickButton(arduinoController2, 7); // 19
        hoodTrenchButton = new JoystickButton(arduinoController2, 8); // 20

        ledTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
        ledEntry = ledTable.getEntry("LEDs");

        // Define LED mapping
        ledMap.put(OILED.MISC_1, 0);
        ledMap.put(OILED.MISC_2, 1);
        ledMap.put(OILED.MISC_3, 2);

        ledEntry.setNumberArray(
                new Number[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
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
    public Trigger getShooterUnstickButton() {
        return shooterUnstickButton;
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
    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }

    @Override
    public void updateLED(OILED led, OILEDState state) {
        if (ledMap.containsKey(led)) {
            Number[] array = ledTable.getEntry("LEDs").getNumberArray(
                    new Number[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
            array[ledMap.get(led)] = state.ordinal();
            ledEntry.setNumberArray(array);
        }
    }
}
