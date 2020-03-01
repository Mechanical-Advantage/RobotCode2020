/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An OI class that provides driving and driver overrides with an XBox style
 * controller.
 */
public class OIHandheldWithOverrides extends OIHandheld implements IDriverOverrideOI {
    private boolean driveEnabled = true;
    private boolean openLoop = false;

    private Trigger openLoopSwitch = new Trigger(() -> openLoop);
    private Trigger driveDisableSwitch = new Trigger(() -> driveEnabled).negate();

    private JoystickButton toggleDriveEnabled;
    private JoystickButton toggleOpenLoop;

    public OIHandheldWithOverrides(int ID) {
        super(ID);

        toggleDriveEnabled = new JoystickButton(driverController, 7); // back button
        toggleOpenLoop = new JoystickButton(driverController, 8); // start button

        // The toggle buttons are not exposed and this class fakes having a disable
        // switch
        toggleDriveEnabled.whenPressed(new InstantCommand() {
            @Override
            public void initialize() {
                driveEnabled = !driveEnabled;
                SmartDashboard.putBoolean("Drive Enabled", driveEnabled);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        toggleOpenLoop.whenPressed(new InstantCommand() {
            @Override
            public void initialize() {
                openLoop = !openLoop;
                SmartDashboard.putBoolean("Open Loop", openLoop);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
    }

    @Override
    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }
}
