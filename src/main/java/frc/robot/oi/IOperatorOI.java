/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterHood.HoodPosition;

/**
 * OI interface for operator controls.
 */
public interface IOperatorOI {

    static final Trigger dummyTrigger = new Trigger();

    public Trigger getShooterFlywheelRunButton();

    public Trigger getShooterFlywheelStopButton();

    public Trigger getIntakeExtendButton();

    public Trigger getIntakeRetractButton();

    public Trigger getRunIntakeForwardsButton();

    public Trigger getRunIntakeBackwardsButton();

    public default Trigger getClimbEnableSwitch() {
        return dummyTrigger;
    }

    public default double getClimbStickY() {
        return 0;
    }

    public default double getClimbStickX() {
        return 0;
    }

    public default Trigger getManualHoodSwitch() {
        return dummyTrigger;
    }

    public default Trigger getHoodWallButton() {
        return dummyTrigger;
    }

    public default Trigger getHoodFrontLineButton() {
        return dummyTrigger;
    }

    public default Trigger getHoodBackLineButton() {
        return dummyTrigger;
    }

    public default Trigger getHoodTrenchButton() {
        return dummyTrigger;
    }

    public default Trigger getGalacticSearchButton() {
        return dummyTrigger;
    }

    public default Trigger getPowerPortAutoButton() {
        return dummyTrigger;
    }

    public default boolean getLockWall() {
        return false;
    }

    public default void updateTimer() {
    }

    public default void setPressure(double pressure) {
    }

    public default void setFlyWheelSpeed(double rpm) {
    }

    public default void setHoodPosition(HoodPosition position) {
    }

    public default void updateLED(OILED led, OILEDState state) {
    }

    public static enum OILED {
        OPEN_LOOP, DRIVE_DISABLE, LIMELIGHT_DISABLE, MANUAL_HOOD, BUDDY_CLIMB, CLIMB_ENABLE, HOOD_WALL, HOOD_FRONT_LINE,
        HOOD_BACK_LINE, HOOD_TRENCH, INTAKE_EXTEND, INTAKE_RETRACT, INTAKE_FORWARD, INTAKE_BACKWARD, SHOOTER_RUN,
        SHOOTER_STOP, SHOOTER_SHOOT, SHOOTER_UNSTICK
    }

    public static enum OILEDState {
        OFF, BLINK_SLOW, BLINK_FAST, PULSE_SLOW, PULSE_FAST, DIM, MED, ON
    }

    /**
     * A functional interface for the update LED method
     */
    @FunctionalInterface
    public interface UpdateLEDInterface {
        void update(OILED led, OILEDState state);
    }

    /**
     * A functional interface for the setPressure() method of operatorOI
     */
    public interface SetFlyWheelSpeedInterface {
        void set(double rpm);
    }

    /**
     * A functional interface for the setHoodPosition() method of operatorOI
     */
    @FunctionalInterface
    public interface SetHoodPositionLCDInterface {
        void set(HoodPosition position);
    }

    /**
     * A functional interface for the setPressure() method of operatorOI
     */
    @FunctionalInterface
    public interface SetPressureInterface {
        void set(double pressure);
    }
}
