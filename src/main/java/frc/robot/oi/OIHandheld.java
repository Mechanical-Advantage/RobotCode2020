package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OIHandheld extends OI {
    private boolean joysticksReversed = false;
    private boolean driveEnabled = true;
    private boolean openLoop = true;

    // map driver controller to ID 0 and operator controller to ID 1 in driver
    // station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    private POVButton joysticksForwards = new POVButton(driverController, 0);
    private POVButton joysticksBackwards = new POVButton(driverController, 180);
    private JoystickButton toggleDriveEnabled = new JoystickButton(driverController, 7); // back button
    private JoystickButton toggleOpenLoop = new JoystickButton(driverController, 8); // start button

    public OIHandheld() {
        CommandScheduler.getInstance().clearButtons();
        resetRumble();
        joysticksForwards.whenPressed(new ReverseJoysticks(false));
        joysticksBackwards.whenPressed(new ReverseJoysticks(true));
        toggleDriveEnabled.whenPressed(new ToggleDriveEnabled());
        toggleOpenLoop.whenPressed(new ToggleOpenLoop());
    }

    @Override
    public double getLeftAxis() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kRight) * -1;
        } else {
            return driverController.getY(Hand.kLeft);
        }
    }

    @Override
    public double getRightAxis() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kLeft) * -1;
        } else {
            return driverController.getY(Hand.kRight);
        }
    }

    @Override
    public double getSingleDriveAxisLeft() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kLeft) * -1;
        } else {
            return driverController.getY(Hand.kLeft);
        }
    }

    @Override
    public double getSingleDriveAxisRight() {
        if (joysticksReversed) {
            return driverController.getY(Hand.kRight) * -1;
        } else {
            return driverController.getY(Hand.kRight);
        }
    }

    @Override
    public double getLeftHorizDriveAxis() {
        return driverController.getX(Hand.kLeft);
    }

    @Override
    public double getRightHorizDriveAxis() {
        return driverController.getX(Hand.kRight);
    }

    @Override
    public void setRumble(OIRumbleType type, double value) {
        value = value > 1 ? 1 : value;
        switch (type) {
        case DRIVER_LEFT:
            driverController.setRumble(RumbleType.kLeftRumble, value);
        case DRIVER_RIGHT:
            driverController.setRumble(RumbleType.kRightRumble, value);
        case OPERATOR_LEFT:
            operatorController.setRumble(RumbleType.kLeftRumble, value);
        case OPERATOR_RIGHT:
            operatorController.setRumble(RumbleType.kRightRumble, value);
        }
    }

    @Override
    public void resetRumble() {
        for (OIRumbleType type : OIRumbleType.values()) {
            setRumble(type, 0);
        }
    }

    @Override
    public boolean getOpenLoop() {
        return openLoop;
    }

    @Override
    public void toggleOpenLoop() {
        openLoop = !openLoop;
    }

    @Override
    public boolean getDriveEnabled() {
        return driveEnabled;
    }

    @Override
    public void toggleDriveEnabled() {
        driveEnabled = !driveEnabled;
    }

    @Override
    public boolean getSniperMode() {
        return driverController.getAButton() || driverController.getBButton() || driverController.getBumper(Hand.kLeft)
                || driverController.getBumper(Hand.kRight);
    }

    @Override
    public boolean getSniperHigh() {
        return driverController.getBButton() || driverController.getBumper(Hand.kRight);
    }

    @Override
    public boolean getSniperLow() {
        return driverController.getAButton() || driverController.getBumper(Hand.kLeft);
    }

    @Override
    public void reverseJoysticks(boolean reverse) {
        joysticksReversed = reverse;
    }

    @Override
    public double getLeftOperatorStickY() {
        return operatorController.getY(Hand.kLeft);
    }

    @Override
    public double getRightOperatorStickY() {
        return operatorController.getY(Hand.kRight);
    }

    @Override
    public double getDeadband() {
        return 0.09;
    }
}