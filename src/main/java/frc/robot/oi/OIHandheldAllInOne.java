/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An OI class that supports using a single handheld controller for everything
 * (driver and operator).
 */
public class OIHandheldAllInOne extends OIHandheldWithOverrides implements IOperatorOI {
    private Button shooterFlywheelRunButton = new Button(driverController::getAButton);
    private Button shooterFlywheelStopButton = new Button(driverController::getBButton);
    private Button shooterRollerButton = new Button(driverController::getXButton);

    private Button intakeExtendButton = new POVButton(driverController, 90);
    private Button intakeRetractButton = new POVButton(driverController, 270);
    private Button intakeForwardsButton = new POVButton(driverController, 0);
    private Button intakeBackwardsButton = new POVButton(driverController, 180);
    private Button climbEnableButton;

    @Override
    public Trigger getShooterFlywheelRunButton() {
        return shooterFlywheelRunButton;
    }

    @Override
    public Trigger getShooterFlywheelStopButton() {
        return shooterFlywheelStopButton;
    }

    @Override
    public Trigger getShooterRollerButton() {
        return shooterRollerButton;
    }

    @Override
    public Trigger getIntakeExtendButton() {
        return intakeExtendButton;
    }

    @Override
    public Trigger getIntakeRetractButton() {
        return intakeRetractButton;
    }

    @Override
    public Trigger getRunIntakeForwardsButton() {
        return intakeForwardsButton;
    }

    @Override
    public Trigger getRunIntakeBackwardsButton() {
        return intakeBackwardsButton;
    }

    @Override
    public Trigger getClimbEnableButton() {
        return climbEnableButton;
    }
}
