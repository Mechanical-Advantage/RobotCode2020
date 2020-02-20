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
 * real Triggers.
 */
public class DummyOI implements IDriverOI, IDriverOverrideOI, IOperatorOI {

    @Override
    public Trigger getOpenLoopSwitch() {
        return IDriverOverrideOI.dummyTrigger;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return IDriverOverrideOI.dummyTrigger.negate();
    }

    @Override
    public Trigger getShooterFlywheelButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public Trigger getShooterRollerButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public Trigger getIntakeExtendButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public Trigger getIntakeRetractButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public Trigger getRunIntakeForwardsButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public Trigger getRunIntakeBackwardsButton() {
        return IOperatorOI.dummyTrigger;
    }

    @Override
    public double getLeftDriveY() {
        return 0;
    }

    @Override
    public double getLeftDriveX() {
        return 0;
    }

    @Override
    public double getRightDriveY() {
        return 0;
    }

    @Override
    public double getRightDriveX() {
        return 0;
    }
}
