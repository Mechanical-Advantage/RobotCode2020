/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.SetCamera;
import frc.robot.subsystems.CameraSystem;
import frc.robot.util.UtilFunctions;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OIConsole extends OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  private static final double sliderMin = 0.3;

  private boolean joysticksReversed = false;

  // map left stick to ID 0 and right to ID 1 in driver station
  private Joystick leftController = new Joystick(0);
  private Joystick rightController = new Joystick(1);
  private Joystick oiController1 = new Joystick(2);
  private Joystick oiController2 = new Joystick(3);

  private Button frontCameraButton = new JoystickButton(rightController, 3);
  private Button rearCameraButton = new JoystickButton(rightController, 2);
  @SuppressWarnings("unused")
  private Button joysticksForward = new JoystickButton(leftController, 3);
  @SuppressWarnings("unused")
  private Button joysticksBackward = new JoystickButton(leftController, 2);
  private Button sniperMode = new JoystickButton(rightController, 1);
  private Button toggleGear = new JoystickButton(leftController, 1);
  private Button openLoopDrive = new JoystickButton(oiController2, 10);
  private Button driveDisableSwitch = new JoystickButton(oiController2, 9);
  private Button shiftDisableSwitch = new JoystickButton(oiController2, 8);
  private Button highGear = new JoystickButton(leftController, 5);
  private Button lowGear = new JoystickButton(leftController, 4);

  NetworkTable ledTable;
  NetworkTableEntry ledEntry;

  public OIConsole(CameraSystem cameraSubsystem) {
    CommandScheduler.getInstance().clearButtons();
    ledTable = NetworkTableInstance.getDefault().getTable("LEDs");
    ledEntry = ledTable.getEntry("OI LEDs");

    frontCameraButton.whenPressed(new SetCamera(cameraSubsystem, true));
    rearCameraButton.whenPressed(new SetCamera(cameraSubsystem, false));
    // joysticksForward.whenPressed(new SetCamera(true));
    // joysticksBackward.whenPressed(new SetCamera(false));
    // joysticksForward.whenPressed(new ReverseJoysticks(false));
    // joysticksBackward.whenPressed(new ReverseJoysticks(true));
    highGear.whenPressed(new SwitchGear(DriveGear.HIGH));
    lowGear.whenPressed(new SwitchGear(DriveGear.LOW));
    toggleGear.whenPressed(new ToggleGear());

    ledEntry.setBooleanArray(new boolean[] { false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false });
  }

  @Override
  public double getLeftAxis() {
    if (joysticksReversed) {
      return rightController.getRawAxis(1) * -1;
    } else {
      return leftController.getRawAxis(1);
    }
  }

  @Override
  public double getRightAxis() {
    if (joysticksReversed) {
      return leftController.getRawAxis(1) * -1;
    } else {
      return rightController.getRawAxis(1);
    }
  }

  // reversing the joysticks should not change which joystick to use for straight
  // drive, use
  // different function to make that correct
  // Note: Brian is left-handed
  @Override
  public double getSingleDriveAxisLeft() {
    if (joysticksReversed) {
      return leftController.getRawAxis(1) * -1;
    } else {
      return leftController.getRawAxis(1);
    }
  }

  @Override
  public double getSingleDriveAxisRight() {
    if (joysticksReversed) {
      return rightController.getRawAxis(1) * -1;
    } else {
      return rightController.getRawAxis(1);
    }
  }

  @Override
  public double getLeftHorizDriveAxis() {
    return leftController.getRawAxis(0);
  }

  @Override
  public double getRightHorizDriveAxis() {
    return rightController.getRawAxis(0);
  }

  @Override
  public boolean getOpenLoop() {
    return openLoopDrive.get();
  }

  @Override
  public boolean getDriveEnabled() {
    return !driveDisableSwitch.get();
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
  public void reverseJoysticks(boolean reverse) {
    joysticksReversed = reverse;
  }

  @Override
  public boolean isShiftingEnabled() {
    return !shiftDisableSwitch.get();
  }

  @Override
  public double getSliderLevel() {
    return UtilFunctions.map(oiController2.getRawAxis(1), -1, 1, sliderMin, 1);
  }

  @Override
  public double getLeftOperatorStickY() {
    return oiController1.getY();
  }

  @Override
  public double getRightOperatorStickY() {
    return oiController1.getY();
  }

  @Override
  public void updateLED(OILED led, boolean state) {
    boolean[] array = ledTable.getEntry("OI LEDs").getBooleanArray(new boolean[] { false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false, false, false });
    array[led.ordinal()] = state;
    ledEntry.setBooleanArray(array);
  }
}
