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
    private Button shooterFlywheelButton = new Button(driverController::getYButton);
    private Button shooterRollerButton = new POVButton(driverController, 270);

    @Override
    public Trigger getShooterFlywheelButton() {
        return shooterFlywheelButton;
    }

    @Override
    public Trigger getShooterRollerButton() {
        return shooterRollerButton;
    }
}
