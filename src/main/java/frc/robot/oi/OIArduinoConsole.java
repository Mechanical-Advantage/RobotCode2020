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
    private JoystickButton extendClimber;

    public OIArduinoConsole(int firstID, int secondID) {
        arduinoController1 = new Joystick(firstID);
        arduinoController2 = new Joystick(secondID);

        extendClimber = new JoystickButton(arduinoController2, 0);

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

    public Trigger getClimbEnableButton() {
        return climbEnableSwitch;
    }

    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }

    @Override
    public Trigger getManualHoodSwitch() {
        return manualHoodSwitch;
    }

    @Override
    public Trigger getHoodWallButton() {
        return hoodWallButton;
    }

    @Override
    public Trigger getHoodLineButton() {
        return hoodLineButton;
    }

    @Override
    public Trigger getHoodTrenchButton() {
        return hoodTrenchButton;
    }
}
