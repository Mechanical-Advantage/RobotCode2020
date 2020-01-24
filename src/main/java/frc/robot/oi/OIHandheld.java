package frc.robot.oi;

import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OIHandheld extends OI {
    private boolean driveEnabled = true;
    private boolean openLoop = true;

    private Trigger openLoopSwitch = new FakeSwitch(false, () -> openLoop);
    private Trigger driveDisableSwitch = new FakeSwitch(true, () -> driveEnabled);

    // map driver controller to ID 0 and operator controller to ID 1 in driver
    // station
    private XboxController driverController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);

    private POVButton joysticksForward = new POVButton(driverController, 0);
    private POVButton joysticksReverse = new POVButton(driverController, 180);
    private JoystickButton toggleDriveEnabled = new JoystickButton(driverController, 7); // back button
    private JoystickButton toggleOpenLoop = new JoystickButton(driverController, 8); // start button

    private Button shooterPrototypeFlywheelButton = new Button(driverController::getYButton);
    private Button shooterPrototypeRollerButton = new POVButton(driverController, 270);

    public OIHandheld() {
        resetRumble();
        // The toggle buttons are not exposed and this class fakes having a disable
        // switch
        toggleDriveEnabled.whenPressed(new InstantCommand(this::toggleDriveEnabled));
        toggleOpenLoop.whenPressed(new InstantCommand(this::toggleOpenLoop));
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
    public Trigger getOpenLoopSwitch() {
        return openLoopSwitch;
    }

    private void toggleOpenLoop() {
        openLoop = !openLoop;
    }

    @Override
    public Trigger getDriveDisableSwitch() {
        return driveDisableSwitch;
    }

    private void toggleDriveEnabled() {
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
    public Trigger getShooterPrototypeFlywheelButton() {
        return shooterPrototypeFlywheelButton;
    }

    @Override
    public Trigger getShooterPrototypeRollerButton() {
        return shooterPrototypeRollerButton;
    }

    @Override
    public double getDeadband() {
        return 0.09;
    }

    /**
     * A trigger that gets its input from a Callable.
     */
    private static class FakeSwitch extends Trigger {

        private boolean invert;
        private Callable<Boolean> input;

        /**
         * Creates a new FakeSwitch
         * 
         * @param invert Whether to invert the input
         * @param input  The Callable used to get the current value
         */
        public FakeSwitch(boolean invert, Callable<Boolean> input) {
            this.invert = invert;
            this.input = input;
        }

        @Override
        public boolean get() {
            try {
                Boolean state = input.call();
                return invert ? !state : state;
            } catch (Exception e) {
                // Since this is just used to retrieve the value of a primitive this block
                // should never run.
                e.printStackTrace();
                // Assume the switch is set
                return true;
            }
        }
    }
}
