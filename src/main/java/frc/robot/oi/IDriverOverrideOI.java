/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * OI interface for driver overrides.
 */
public interface IDriverOverrideOI {

    static final Trigger dummyTrigger = new Trigger();

    public Trigger getOpenLoopSwitch();

    public Trigger getDriveDisableSwitch();

    public default Trigger getShiftLockSwitch() {
        return dummyTrigger;
    }

    public default Trigger getLimelightLEDDisableSwitch() {
        return dummyTrigger;
    }
}
