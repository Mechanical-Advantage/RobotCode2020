/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI interface for driver controls.
 */
public interface IDriverOI {
    public static final double minAcceleration = 0.2; // Minimum total horizontal acceleration before rumbling
                                                      // controller
    public static final double fullAcceleration = 0.8; // Total horizontal acceleration (g) for full high frequency
                                                       // rumble
    public static final double lowRumbleFactor = 0.15; // Multiplied by high frequency rumble power to calculate low
                                                       // frequency rumble power
    static final Trigger dummyTrigger = new Trigger();

    public double getLeftDriveY();

    public double getLeftDriveX();

    public double getRightDriveY();

    public double getRightDriveX();

    public default double getLeftDriveTrigger() {
        return 0;
    }

    public default double getRightDriveTrigger() {
        return 0;
    }

    public default boolean hasDriveTriggers() {
        return false;
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

    public default Trigger getAutoAimButton() {
        return dummyTrigger;
    }

    public default Trigger getBackupFromWallButton() {
        return dummyTrigger;
    }

    public default Trigger getAutoDriveButton() {
        return dummyTrigger;
    }

    public default Trigger getVisionTestButton() {
        return dummyTrigger;
    }

    public default void setDriverRumble(DriverOIRumbleType type, double value) {
    }

    public default void resetRumble() {
    }

    public default double getDeadband() {
        return 0;
    }

    public static enum DriverOIRumbleType {
        LEFT, RIGHT
    }
}
