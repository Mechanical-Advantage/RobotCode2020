package frc.robot.oi;

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

    public double getLeftAxis() {
        return 0;
    }

    public double getRightAxis() {
        return 0;
    }

    public double getSingleDriveAxisLeft() {
        return 0;
    }

    public double getSingleDriveAxisRight() {
        return 0;
    }

    public double getLeftHorizDriveAxis() {
        return 0;
    }

    public double getRightHorizDriveAxis() {
        return 0;
    }

    public boolean getOpenLoop() {
        return false;
    }

    public void toggleOpenLoop() {
    };

    public boolean getDriveEnabled() {
        return false;
    }

    public void toggleDriveEnabled() {
    };

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

    public void reverseJoysticks(boolean reverse) {
    }

    public boolean isShiftingEnabled() {
        return false;
    }

    public double getSliderLevel() {
        return 0;
    }

    public void setRumble(OIRumbleType type, double value) {
    }

    public void resetRumble() {
    }

    public double getLeftOperatorStickY() {
        return 0;
    }

    public double getRightOperatorStickY() {
        return 0;
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

    public static enum OIType {
        CONSOLE, HANDHELD
    }
}
