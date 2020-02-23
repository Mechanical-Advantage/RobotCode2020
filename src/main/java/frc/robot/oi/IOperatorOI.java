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

    public default void updateLED(OILED led, boolean state) {
    }

    public static enum OILED {
        MISC_1, MISC_2, MISC_3, INTAKE_RETRACT, INTAKE_ON_OFF, VAC_PICKUP, VAC_TAIL, TOGGLE_LOW, TOGGLE_HIGH,
        JOYSTICK_YELLOW, ARM_ALT, ARM_FLOOR, ARM_CARGO_SHIP, ARM_ROCKET_LOW, ARM_ROCKET_MID, ARM_ROCKET_HIGH, ARM_HOME
    }
}
