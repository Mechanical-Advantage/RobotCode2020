package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Acts as an interface to multiple OI configurations
 */
public interface OI {
    public static final double minAcceleration = 0.2; // Minimum total horizontal acceleration before rumbling
                                                      // controller
    public static final double fullAcceleration = 0.8; // Total horizontal acceleration (g) for full high frequency
                                                       // rumble
    public static final double lowRumbleFactor = 0.15; // Multiplied by high frequency rumble power to calculate low
                                                       // frequency rumble power
    static final Trigger dummyTrigger = new Trigger();

    public default double getLeftDriveY() {
        return 0;
    }

    public default double getLeftDriveX() {
        return 0;
    }

    public default double getRightDriveY() {
        return 0;
    }

    public default double getRightDriveX() {
        return 0;
    }

    public default double getLeftDriveTrigger() {
        return 0;
    }

    public default double getRightDriveTrigger() {
        return 0;
    }

    public default boolean hasDriveTriggers() {
        return false;
    }

    // All OIs are required to implement an open loop switch and drive disable.

    public Trigger getOpenLoopSwitch();

    public Trigger getDriveDisableSwitch();

    public default Trigger getShiftLockSwitch() {
        return dummyTrigger;
    }

    public default boolean getSniperMode() {
        return false;
    }

    public default double getSniperLevel() {
        return 0;
    }

    public default double getSniperHighLevel() {
        return 0;
    }

    public default double getSniperLowLevel() {
        return 0;
    }

    public default boolean getSniperHigh() {
        return false;
    }

    public default boolean getSniperLow() {
        return false;
    }

    public default boolean hasDualSniperMode() {
        return false;
    }

    public default Trigger getShiftDisableSwitch() {
        return dummyTrigger;
    }

    public default Trigger getHighGearButton() {
        return dummyTrigger;
    }

    public default Trigger getLowGearButton() {
        return dummyTrigger;
    }

    public default Trigger getToggleGearButton() {
        return dummyTrigger;
    }

    public default Trigger getJoysticksForwardButton() {
        return dummyTrigger;
    }

    public default Trigger getJoysticksReverseButton() {
        return dummyTrigger;
    }

    public default Trigger getFrontCameraButton() {
        return dummyTrigger;
    }

    public default Trigger getSecondCameraButton() {
        return dummyTrigger;
    }

    public default Trigger getVisionTestButton() {
        return dummyTrigger;
    }

    public default Trigger getShooterPrototypeFlywheelButton() {
        return dummyTrigger;
    }

    public default Trigger getShooterPrototypeRollerButton() {
        return dummyTrigger;
    }

    public default Trigger getAutoAimButton() {
        return dummyTrigger;
    }

    public default Trigger getAutoDriveButton() {
        return dummyTrigger;
    }

    public default void setRumble(OIRumbleType type, double value) {
    }

    public default void resetRumble() {
    }

    public default double getDeadband() {
        return 0;
    }

    public default void updateLED(OILED led, boolean state) {
    }

    public static enum OILED {
        MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
        JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
    }

    public static enum OIRumbleType {
        DRIVER_LEFT, DRIVER_RIGHT, OPERATOR_LEFT, OPERATOR_RIGHT
    }
}
