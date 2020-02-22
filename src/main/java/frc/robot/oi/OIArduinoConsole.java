/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

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

    private Button openLoopSwitch = new JoystickButton(arduinoController1, 1);
    private Button driveDisableSwitch = new JoystickButton(arduinoController1, 2);
    private Button manualHoodSwitch = new JoystickButton(arduinoController1, 4);
    private Button buddyClimbSwitch = new JoystickButton(arduinoController1, 5);
    private Button climbEnableSwitch = new JoystickButton(arduinoController1, 6);

    private Button intakeExtendButton = new JoystickButton(arduinoController1, 7);
    private Button intakeRetractButton = new JoystickButton(arduinoController1, 8);
    private Button intakeForwardsButton = new JoystickButton(arduinoController1, 9);
    private Button intakeBackwardsButton = new JoystickButton(arduinoController1, 10);

    private Button shooterFlywheelRunButton = new JoystickButton(arduinoController2, 2); // 14
    private Button shooterFlywheelStopButton = new JoystickButton(arduinoController2, 1); // 13
    private Button shooterRollerButton = new JoystickButton(arduinoController2, 5); // 17

    private Button hoodWallButton = new JoystickButton(arduinoController2, 6); // 18
    private Button hoodLineButton = new JoystickButton(arduinoController2, 7); // 19
    private Button hoodTrenchButton = new JoystickButton(arduinoController2, 8); // 20

    public OIArduinoConsole(int firstID, int secondID) {
        arduinoController1 = new Joystick(firstID);
        arduinoController2 = new Joystick(secondID);
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
    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }

}
