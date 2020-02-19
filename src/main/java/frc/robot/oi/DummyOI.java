/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An OI class that can be instantiated when the real OI is unknown to provide
 * real Triggers
 */
public class DummyOI implements OI {

    @Override
    public Trigger getOpenLoopSwitch() {
        return dummyTrigger;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return dummyTrigger.negate();
    }
}
