package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Acts as an interface to multiple OI configurations
 */
public abstract class OI {
    public static final double minAcceleration = 0.2; // Minimum total horizontal acceleration before rumbling
                                                      // controller
    public static final double fullAcceleration = 0.8; // Total horizontal acceleration (g) for full high frequency
                                                       // rumble
    public static final double lowRumbleFactor = 0.15; // Multiplied by high frequency rumble power to calculate low
                                                       // frequency rumble power
    static final Trigger dummyTrigger = new Trigger();

    public double getLeftDriveY() {
        return 0;
    }

    public double getLeftDriveX() {
        return 0;
    }

    public double getRightDriveY() {
        return 0;
    }

    public double getRightDriveX() {
        return 0;
    }

    public double getLeftDriveTrigger() {
        return 0;
    }

    public double getRightDriveTrigger() {
        return 0;
    }

    public boolean hasDriveTriggers() {
        return false;
    }

    // All OIs are required to implement an open loop switch and drive disable.

    public abstract Trigger getOpenLoopSwitch();

    public abstract Trigger getDriveDisableSwitch();

    public Trigger getShiftLockSwitch() {
        return dummyTrigger;
    }

    public boolean getSniperMode() {
        return false;
    }

    public double getSniperLevel() {
        return 0;
    }

    public boolean getSniperHigh() {
        return false;
    }

    public boolean getSniperLow() {
        return false;
    }

    public boolean hasDualSniperMode() {
        return false;
    }

    public Trigger getShiftDisableSwitch() {
        return dummyTrigger;
    }

    public Trigger getHighGearButton() {
        return dummyTrigger;
    }

    public Trigger getLowGearButton() {
        return dummyTrigger;
    }

    public Trigger getToggleGearButton() {
        return dummyTrigger;
    }

    public Trigger getJoysticksForwardButton() {
        return dummyTrigger;
    }

    public Trigger getJoysticksReverseButton() {
        return dummyTrigger;
    }

    public Trigger getFrontCameraButton() {
        return dummyTrigger;
    }

    public Trigger getSecondCameraButton() {
        return dummyTrigger;
    }

    public Trigger getVisionTestButton() {
        return dummyTrigger;
    }

    public void setRumble(OIRumbleType type, double value) {
    }

    public void resetRumble() {
    }

    public double getDeadband() {
        return 0;
    }

    public void updateLED(OILED led, boolean state) {
    }

    public static enum OILED {
        MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
        JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
    }

    public static enum OIRumbleType {
        DRIVER_LEFT, DRIVER_RIGHT, OPERATOR_LEFT, OPERATOR_RIGHT
    }
}
