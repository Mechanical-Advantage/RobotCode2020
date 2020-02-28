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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.util.OILCDField;

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

    NetworkTable ledTable;
    NetworkTableEntry ledEntry;

    OILCDField LCDTimer;
    OILCDField LCDPressure;
    OILCDField LCDFlyWheelSpeed;
    OILCDField LCDHoodPosition;

    private static final Map<OILED, Integer> ledMap = new HashMap<OILED, Integer>();

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

        // Set up LCD fields
        OILCDField.clearFields();
        LCDTimer = new OILCDField("Timer", 0, 0, 20, "   Unknown - ?:??");
        LCDPressure = new OILCDField("Pressure", 0, 1, 20, "Pressure: ?? PSI");
        LCDFlyWheelSpeed = new OILCDField("FlyWheelSpeed", 0, 2, 20, "Flywheel: ???? RPM");
        LCDHoodPosition = new OILCDField("HoodPosition", 0, 3, 20, "Hood: Unknown");

        // Set up LED entry
        ledTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
        ledEntry = ledTable.getEntry("LEDs");

        // Define LED mapping
        ledMap.put(OILED.OPEN_LOOP, 0);
        ledMap.put(OILED.DRIVE_DISABLE, 1);
        ledMap.put(OILED.MANUAL_HOOD, 3);
        ledMap.put(OILED.BUDDY_CLIMB, 4);
        ledMap.put(OILED.CLIMB_ENABLE, 5);
        ledMap.put(OILED.HOOD_BOTTOM, 7);
        ledMap.put(OILED.HOOD_MIDDLE, 8);
        ledMap.put(OILED.HOOD_TOP, 9);
        ledMap.put(OILED.INTAKE_EXTEND, 10);
        ledMap.put(OILED.INTAKE_RETRACT, 11);
        ledMap.put(OILED.INTAKE_FORWARD, 12);
        ledMap.put(OILED.INTAKE_BACKWARD, 13);
        ledMap.put(OILED.SHOOTER_STOP, 16);
        ledMap.put(OILED.SHOOTER_RUN, 17);
        ledMap.put(OILED.SHOOTER_UNSTICK, 18);
        ledMap.put(OILED.SHOOTER_SHOOT, 19);

        ledEntry.setNumberArray(new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
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
    public Trigger getClimbEnableSwitch() {
        return climbEnableSwitch;
    }

    @Override
    public double getClimbStickY() {
        return arduinoController1.getY();
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

    @Override
    public void updateTimer() {
        int time = (int) DriverStation.getInstance().getMatchTime();
        String timeString = "?:??";
        if (time >= 0) {
            int minutes = Math.floorDiv(time, 60);
            int seconds = time - (minutes * 60);
            String secondsText = Integer.toString(seconds);
            if (secondsText.length() < 2) {
                secondsText = "0" + secondsText;
            }
            timeString = Integer.toString(minutes) + ":" + secondsText;
        }

        String mode = "Teleop";
        if (DriverStation.getInstance().isDisabled()) {
            mode = "Disabled";
        } else if (DriverStation.getInstance().isAutonomous()) {
            mode = "Auto";
        }

        String fullText = mode + " - " + timeString;
        int totalPadding = 20 - fullText.length();
        if (totalPadding < 0) {
            totalPadding = 0;
        }
        int leftPadding = Math.floorDiv(totalPadding, 2);
        String padding = "";
        for (int i = 0; i < leftPadding; i++) {
            padding = " " + padding;
        }
        LCDTimer.setValue(padding + fullText);
    }

    @Override
    public void setPressure(double pressure) {
        LCDPressure.setValue("Pressure: " + Integer.toString(Math.round((float) pressure)) + " PSI");
    }

    @Override
    public void setFlyWheelSpeed(double rpm) {
        LCDFlyWheelSpeed.setValue("Flywheel: " + Integer.toString(Math.round((float) rpm)) + " RPM");
    }

    @Override
    public void setHoodPosition(HoodPosition position) {
        String text;
        switch (position) {
            case BOTTOM:
                text = "Wall";
                break;
            case MIDDLE:
                text = "Line";
                break;
            case TOP:
                text = "Trench";
                break;
            default:
                text = "Unknown";
                break;
        }
        LCDHoodPosition.setValue("Hood: " + text);
    }

    @Override
    public void updateLED(OILED led, OILEDState state) {
        if (ledMap.containsKey(led)) {
            Double[] array = (Double[]) ledTable.getEntry("LEDs").getNumberArray(
                    new Integer[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
            array[ledMap.get(led)] = (double) state.ordinal();
            ledEntry.setNumberArray(array);
        }
    }
}
