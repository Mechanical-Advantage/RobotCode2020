/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An OI class that provides driving support using two Attack 3 style
 * controllers.
 */
public class OIDualJoysticks implements IDriverOI {
    // map left stick to ID 0 and right to ID 1 in driver station
    private Joystick leftController = new Joystick(0);
    private Joystick rightController = new Joystick(1);

    private Button frontCameraButton = new JoystickButton(rightController, 3);
    private Button secondCameraButton = new JoystickButton(rightController, 2);
    private Button joysticksForward = new JoystickButton(leftController, 3);
    private Button joysticksReverse = new JoystickButton(leftController, 2);
    private Button sniperMode = new JoystickButton(rightController, 1);
    private Button toggleGear = new JoystickButton(leftController, 1);
    private Button highGear = new JoystickButton(leftController, 5);
    private Button lowGear = new JoystickButton(leftController, 4);

    private Button visionTestButton = new JoystickButton(rightController, 8);

    @Override
    public double getLeftDriveY() {
        return leftController.getRawAxis(1);
    }

    @Override
    public double getLeftDriveX() {
        return leftController.getRawAxis(0);
    }

    @Override
    public double getRightDriveY() {
        return rightController.getRawAxis(1);
    }

    @Override
    public double getRightDriveX() {
        return rightController.getRawAxis(0);
    }

    @Override
    public boolean getSniperMode() {
        return sniperMode.get();
    }

    @Override
    public double getSniperLevel() {
        double sniperLimit = 0.5;
        return (1 - ((rightController.getRawAxis(2) + 1) / 2)) * sniperLimit; // control returns -1 to 1, scale to 0 to
        // 1, subtract from 1 so 1 is up
    }

    @Override
    public Trigger getHighGearButton() {
        return highGear;
    }

    @Override
    public Trigger getLowGearButton() {
        return lowGear;
    }

    @Override
    public Trigger getToggleGearButton() {
        return toggleGear;
    }

    @Override
    public Trigger getJoysticksForwardButton() {
        return joysticksForward;
    }

    @Override
    public Trigger getJoysticksReverseButton() {
        return joysticksReverse;
    }

    @Override
    public Trigger getFrontCameraButton() {
        return frontCameraButton;
    }

    @Override
    public Trigger getSecondCameraButton() {
        return secondCameraButton;
    }

    @Override
    public Trigger getVisionTestButton() {
        return visionTestButton;
    }
}
