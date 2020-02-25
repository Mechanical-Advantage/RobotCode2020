/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI interface for operator controls.
 */
public interface IOperatorOI {

    static final Trigger dummyTrigger = new Trigger();

    public Trigger getShooterFlywheelRunButton();

    public Trigger getShooterFlywheelStopButton();

    public Trigger getShooterRollerButton();

    public Trigger getShooterUnstickButton();

    public Trigger getIntakeExtendButton();

    public Trigger getIntakeRetractButton();

    public Trigger getRunIntakeForwardsButton();

    public Trigger getRunIntakeBackwardsButton();

    public default Trigger getManualHoodSwitch() {
        return dummyTrigger;
    }

    public default Trigger getHoodWallButton() {
        return dummyTrigger;
    }

    public default Trigger getHoodLineButton() {
        return dummyTrigger;
    }

    public default Trigger getHoodTrenchButton() {
        return dummyTrigger;
    }

    public default void setTimer(int timeRemaining) {
    }

    public default void setPressure(double pressure) {
    }

    public default void setFlyWheelSpeed(double rpm) {
    }

    public default void updateLED(OILED led, OILEDState state) {
    }

    public static enum OILED {
        OPEN_LOOP, DRIVE_DISABLE, MANUAL_HOOD, BUDDY_CLIMB, CLIMB_ENABLE, INTAKE_EXTEND, INTAKE_RETRACT, INTAKE_FORWARD,
        INTAKE_BACKWARD, SHOOTER_RUN, SHOOTER_STOP, SHOOTER_SHOOT, SHOOTER_UNSTICK
    }

    public static enum OILEDState {
        OFF, BLINK_SLOW, BLINK_FAST, PULSE_SLOW, PULSE_FAST, DIM, MED, ON
    }
}
