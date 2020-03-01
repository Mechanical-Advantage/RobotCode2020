/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Set of operator controls for an XBox style controller
 */
public class OIOperatorHandheld implements IOperatorOI {
    private XboxController controller;

    private Button shooterFlywheelRunButton;
    private Button shooterFlywheelStopButton;
    private Button shooterRollerButton;
    private Button shooterUnstickButton;

    private Button intakeExtendButton;
    private Button intakeRetractButton;
    private Button intakeForwardsButton;
    private Button intakeBackwardsButton;

    private Button climbEnableButton;
    private Button climbDisableButton;
    private Trigger fakeClimbEnableSwitch;

    private Button wallButton;
    private Button lineButton;
    private Button trenchButton;
    private Trigger fakeManualHoodSwitch = dummyTrigger.negate();

    private boolean climbEnabled;

    public OIOperatorHandheld(int ID) {
        controller = new XboxController(ID);

        shooterFlywheelRunButton = new Button(controller::getAButton);
        shooterFlywheelStopButton = new Button(controller::getBButton);
        shooterRollerButton = new Button(controller::getXButton);
        shooterUnstickButton = new Button(controller::getYButton);

        intakeExtendButton = new Button(() -> controller.getBumper(Hand.kRight));
        intakeRetractButton = new Button(() -> controller.getBumper(Hand.kLeft));
        ;
        intakeForwardsButton = new Button(() -> controller.getTriggerAxis(Hand.kRight) > 0.5);
        intakeBackwardsButton = new Button(() -> controller.getTriggerAxis(Hand.kLeft) > 0.5);

        climbEnableButton = new Button(controller::getStartButton);
        climbEnableButton.whenPressed(() -> climbEnabled = true);
        climbDisableButton = new Button(controller::getBackButton);
        climbDisableButton.whenPressed(() -> climbEnabled = false);
        fakeClimbEnableSwitch = new Trigger(() -> climbEnabled);

        wallButton = new POVButton(controller, 0);
        lineButton = new POVButton(controller, 270);
        trenchButton = new POVButton(controller, 180);
    }

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
    public Trigger getShooterUnstickButton() {
        return shooterUnstickButton;
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
    public Trigger getClimbEnableSwitch() {
        return fakeClimbEnableSwitch;
    }

    @Override
    public double getClimbStickY() {
        return controller.getY(Hand.kRight);
    }

    @Override
    public double getClimbStickX() {
        return controller.getX(Hand.kRight);
    }

    @Override
    public Trigger getHoodWallButton() {
        return wallButton;
    }

    @Override
    public Trigger getHoodLineButton() {
        return lineButton;
    }

    @Override
    public Trigger getHoodTrenchButton() {
        return trenchButton;
    }

    @Override
    public Trigger getManualHoodSwitch() {
        return fakeManualHoodSwitch;
    }
}
