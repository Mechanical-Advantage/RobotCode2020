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

    public Trigger getIntakeExtendButton();

    public Trigger getIntakeRetractButton();

    public Trigger getRunIntakeForwardsButton();

    public Trigger getRunIntakeBackwardsButton();

    public default void updateLED(OILED led, OILEDState state) {
    }

    public static enum OILED {
        MISC_1, MISC_2, MISC_3,
    }

    public static enum OILEDState {
        OFF, BLINK_SLOW, BLINK_FAST, PULSE_SLOW, PULSE_FAST, DIM, MED, ON
    }
}
