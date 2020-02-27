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
    private Joystick leftController;
    private Joystick rightController;

    private Button frontCameraButton;
    private Button secondCameraButton;
    private Button joysticksForward;
    private Button joysticksReverse;
    private Button sniperMode;
    private Button toggleGear;
    private Button highGear;
    private Button lowGear;

    private Button visionTestButton;

    public OIDualJoysticks(int leftID, int rightID) {
        leftController = new Joystick(leftID);
        rightController = new Joystick(rightID);

        frontCameraButton = new JoystickButton(rightController, 3);
        secondCameraButton = new JoystickButton(rightController, 2);
        joysticksForward = new JoystickButton(leftController, 3);
        joysticksReverse = new JoystickButton(leftController, 2);
        sniperMode = new JoystickButton(rightController, 1);
        toggleGear = new JoystickButton(leftController, 1);
        highGear = new JoystickButton(leftController, 5);
        lowGear = new JoystickButton(leftController, 4);

        visionTestButton = new JoystickButton(rightController, 8);
    }

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
