package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Driver OI class for an XBox style controller.
 */
public class OIHandheld implements IDriverOI {
    XboxController driverController = new XboxController(0);

    private POVButton joysticksForward = new POVButton(driverController, 0);
    private POVButton joysticksReverse = new POVButton(driverController, 180);

    private static final double sniperHighLevel = 0.3; // used for right trigger when using handheld control
    private static final double sniperLowLevel = 0.15; // used for left trigger when using handheld control

    private Button autoAimButton = new Button(driverController::getXButton);
    private Button autoDriveButton = new POVButton(driverController, 90);

    public OIHandheld() {
        resetRumble();
    }

    @Override
    public double getLeftDriveY() {
        return driverController.getY(Hand.kLeft);
    }

    @Override
    public double getLeftDriveX() {
        return driverController.getX(Hand.kLeft);
    }

    @Override
    public double getLeftDriveTrigger() {
        return driverController.getTriggerAxis(Hand.kLeft);
    }

    @Override
    public double getRightDriveY() {
        return driverController.getY(Hand.kRight);
    }

    @Override
    public double getRightDriveX() {
        return driverController.getX(Hand.kRight);
    }

    @Override
    public double getRightDriveTrigger() {
        return driverController.getTriggerAxis(Hand.kRight);
    }

    @Override
    public boolean hasDriveTriggers() {
        return true;
    }

    @Override
    public void setDriverRumble(DriverOIRumbleType type, double value) {
        value = value > 1 ? 1 : value;
        switch (type) {
        case LEFT:
            driverController.setRumble(RumbleType.kLeftRumble, value);
            break;
        case RIGHT:
            driverController.setRumble(RumbleType.kRightRumble, value);
            break;
        }
    }

    @Override
    public void resetRumble() {
        for (DriverOIRumbleType type : DriverOIRumbleType.values()) {
            setDriverRumble(type, 0);
        }
    }

    @Override
    public boolean getSniperMode() {
        return getSniperHigh() || getSniperLow();
    }

    @Override
    public double getSniperHighLevel() {
        return sniperHighLevel;
    }

    @Override
    public double getSniperLowLevel() {
        return sniperLowLevel;
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
    public boolean hasDualSniperMode() {
        return true;
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
    public Trigger getAutoAimButton() {
        return autoAimButton;
    }

    @Override
    public Trigger getAutoDriveButton() {
        return autoDriveButton;
    }

    @Override
    public double getDeadband() {
        return 0.09;
    }
}
