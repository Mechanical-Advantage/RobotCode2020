/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

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
    private Button limelightLEDDisableSwitch;

    private Button intakeExtendButton;
    private Button intakeRetractButton;
    private Button intakeForwardsButton;
    private Button intakeBackwardsButton;

    private Button shooterFlywheelRunButton;
    private Button shooterFlywheelStopButton;
    // private Button shooterRollerButton;
    // private Button shooterUnstickButton;

    private Button hoodWallButton;
    private Button hoodFrontLineButton;
    private Button hoodBackLineButton;
    private Button hoodTrenchButton;

    NetworkTable ledTable;
    NetworkTableEntry ledEntry;

    OILCDField LCDTimer;
    OILCDField LCDPressure;
    OILCDField LCDFlyWheelSpeed;
    OILCDField LCDHoodPosition;

    private static final Map<OILED, Integer> ledMap = Map.ofEntries(Map.entry(OILED.OPEN_LOOP, 0),
            Map.entry(OILED.DRIVE_DISABLE, 1), Map.entry(OILED.LIMELIGHT_DISABLE, 2), Map.entry(OILED.MANUAL_HOOD, 3),
            Map.entry(OILED.BUDDY_CLIMB, 4), Map.entry(OILED.CLIMB_ENABLE, 5), Map.entry(OILED.HOOD_WALL, 6),
            Map.entry(OILED.HOOD_FRONT_LINE, 7), Map.entry(OILED.HOOD_BACK_LINE, 8), Map.entry(OILED.HOOD_TRENCH, 9),
            Map.entry(OILED.INTAKE_EXTEND, 10), Map.entry(OILED.INTAKE_RETRACT, 11),
            Map.entry(OILED.INTAKE_FORWARD, 12), Map.entry(OILED.INTAKE_BACKWARD, 13),
            Map.entry(OILED.SHOOTER_STOP, 16), Map.entry(OILED.SHOOTER_RUN, 17), Map.entry(OILED.SHOOTER_UNSTICK, 18),
            Map.entry(OILED.SHOOTER_SHOOT, 19));

    public OIArduinoConsole(int firstID, int secondID) {
        arduinoController1 = new Joystick(firstID);
        arduinoController2 = new Joystick(secondID);

        openLoopSwitch = new JoystickButton(arduinoController1, 1);
        driveDisableSwitch = new JoystickButton(arduinoController1, 2);
        manualHoodSwitch = new JoystickButton(arduinoController1, 4);
        buddyClimbSwitch = new JoystickButton(arduinoController1, 5);
        climbEnableSwitch = new JoystickButton(arduinoController1, 6);
        limelightLEDDisableSwitch = new JoystickButton(arduinoController1, 3);

        intakeExtendButton = new JoystickButton(arduinoController1, 7);
        intakeRetractButton = new JoystickButton(arduinoController1, 8);
        intakeForwardsButton = new JoystickButton(arduinoController1, 9);
        intakeBackwardsButton = new JoystickButton(arduinoController1, 10);

        shooterFlywheelRunButton = new JoystickButton(arduinoController2, 2); // 14
        shooterFlywheelStopButton = new JoystickButton(arduinoController2, 1); // 13
        // shooterRollerButton = new JoystickButton(arduinoController2, 5); // 17
        // shooterUnstickButton = new JoystickButton(arduinoController2, 3); // 15

        hoodWallButton = new JoystickButton(arduinoController2, 5); // 17
        hoodFrontLineButton = new JoystickButton(arduinoController2, 6); // 18
        hoodBackLineButton = new JoystickButton(arduinoController2, 7); // 19
        hoodTrenchButton = new JoystickButton(arduinoController2, 8); // 20

        // Set up LCD fields
        NetworkTableInstance.getDefault().getTable("OperatorInterface").delete("LCD");
        LCDTimer = new OILCDField("Timer", 0, 0, 20, "   Unknown - ?:??");
        LCDPressure = new OILCDField("Pressure", 0, 1, 20, "Pressure: ?? PSI");
        LCDFlyWheelSpeed = new OILCDField("FlyWheelSpeed", 0, 2, 20, "Flywheel: ???? RPM");
        LCDHoodPosition = new OILCDField("HoodPosition", 0, 3, 20, "Hood: Unknown");

        // Set up LED entry
        ledTable = NetworkTableInstance.getDefault().getTable("OperatorInterface");
        ledEntry = ledTable.getEntry("LEDs");

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

    /*
     * @Override public Trigger getShooterRollerButton() { return
     * shooterRollerButton; }
     * 
     * @Override public Trigger getShooterUnstickButton() { return
     * shooterUnstickButton; }
     */

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
        return arduinoController1.getY() * -1;
    }

    @Override
    public double getClimbStickX() {
        return arduinoController1.getX();
    }

    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }

    @Override
    public Trigger getLimelightLEDDisableSwitch() {
        return limelightLEDDisableSwitch;
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
    public Trigger getHoodFrontLineButton() {
        return hoodFrontLineButton;
    }

    @Override
    public Trigger getHoodBackLineButton() {
        return hoodBackLineButton;
    }

    @Override
    public Trigger getHoodTrenchButton() {
        return hoodTrenchButton;
    }

    @Override
    public void updateTimer() {
        int time = (int) DriverStation.getMatchTime();
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
        if (DriverStation.isDisabled()) {
            mode = "Disabled";
        } else if (DriverStation.isAutonomous()) {
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
        LCDPressure.setValue("Pressure: " + Integer.toString(pressure < 0 ? 0 : Math.round((float) pressure)) + " PSI");
    }

    @Override
    public void setFlyWheelSpeed(double rpm) {
        LCDFlyWheelSpeed.setValue("Flywheel: " + Integer.toString(Math.round((float) rpm)) + " RPM");
    }

    @Override
    public void setHoodPosition(HoodPosition position) {
        String text;
        switch (position) {
        case WALL:
            text = "Wall";
            break;
        case FRONT_LINE:
            text = "Front Line";
            break;
        case BACK_LINE:
            text = "Back Line";
            break;
        case TRENCH:
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

    /**
     * Represents a field for the LCD on the operator panel
     */
    public class OILCDField {

        private final NetworkTable table;
        private final int x;
        private final int y;
        private final int length;
        private String value;

        /**
         * Initializes field parameters and creates table
         * 
         * @param key          Arbitrary field identifier (not visible on LCD)
         * @param x            X location of this field (0 is left)
         * @param y            location a/k/a line of this field (0 is top)
         * @param lengthLength Length of this field (will be truncated)
         * @param initialValue Initial text to display
         */
        public OILCDField(String key, int x, int y, int length, String initialValue) {
            table = NetworkTableInstance.getDefault().getTable("OperatorInterface/LCD/" + key);
            this.x = x;
            this.y = y;
            this.length = length;
            value = initialValue;
            updateTable();
        }

        /**
         * Updates the field text
         * 
         * @param value New text to display
         */
        public void setValue(String value) {
            this.value = value;
            updateTable();
        }

        private void updateTable() {
            table.getEntry("X").setNumber(x);
            table.getEntry("Y").setNumber(y);
            table.getEntry("Length").setNumber(length);
            table.getEntry("String").setString(value);
        }
    }
}
