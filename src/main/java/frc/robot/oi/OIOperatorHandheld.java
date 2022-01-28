/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.DoublePressTrigger;

/**
 * Set of operator controls for an XBox style controller
 */
public class OIOperatorHandheld implements IOperatorOI {
    private XboxController controller;

    private Button shooterFlywheelRunButton;
    private Button shooterFlywheelStopButton;
    // private Button shooterRollerButton;
    // private Button shooterUnstickButton;

    private Button intakeExtendButton;
    private Button intakeRetractButton;
    private Button intakeForwardsButton;
    private Button intakeBackwardsButton;

    private Trigger climbEnableButton;
    private Trigger climbDisableButton;
    private Trigger fakeClimbEnableSwitch;

    private Button wallButton;
    private Button frontLineButton;
    private Button backLineButton;
    private Button trenchButton;
    private Button manualHoodButton;
    private Button autoHoodButton;
    private Trigger fakeManualHoodSwitch;

    private boolean climbEnabled = false;
    private boolean manualHood = false;

    private Trigger galacticSearchButton;
    private Trigger powerPortAutoButton;

    public OIOperatorHandheld(int ID) {
        controller = new XboxController(ID);

        shooterFlywheelRunButton = new Button(controller::getAButton);
        shooterFlywheelStopButton = new Button(controller::getBButton);
        // shooterRollerButton = new Button(controller::getXButton);
        // shooterUnstickButton = new Button(controller::getYButton);

        intakeExtendButton = new Button(() -> controller.getRightBumper());
        intakeRetractButton = new Button(() -> controller.getLeftBumper());
        intakeForwardsButton = new Button(() -> controller.getRightTriggerAxis() > 0.5);
        intakeBackwardsButton = new Button(() -> controller.getLeftTriggerAxis() > 0.5);

        climbEnableButton = new DoublePressTrigger(new Button(controller::getXButton));
        climbEnableButton.whenActive(() -> climbEnabled = true);
        climbDisableButton = new Button(controller::getYButton);
        climbDisableButton.whenActive(() -> climbEnabled = false);

        manualHoodButton = new Button(controller::getStartButton);
        manualHoodButton.whenPressed(() -> manualHood = true);
        autoHoodButton = new Button(controller::getBackButton);
        autoHoodButton.whenPressed(() -> manualHood = false);

        fakeClimbEnableSwitch = new Trigger(() -> climbEnabled);
        fakeManualHoodSwitch = new Trigger(() -> manualHood);

        wallButton = new POVButton(controller, 0);
        frontLineButton = new POVButton(controller, 270);
        backLineButton = new POVButton(controller, 90);
        trenchButton = new POVButton(controller, 180);

        galacticSearchButton = new Button(() -> controller.getLeftBumper())
                .and(new Button(() -> controller.getRightBumper()));
        powerPortAutoButton = new Trigger(() -> controller.getLeftY() < -0.5);
    }

    @Override
    public Trigger getShooterFlywheelRunButton() {
        return shooterFlywheelRunButton;
    }

    @Override
    public Trigger getShooterFlywheelStopButton() {
        return shooterFlywheelStopButton;
    }

    /*
     * @Override public Trigger getShooterRollerButton() { return
     * shooterRollerButton; }
     * 
     * @Override public Trigger getShooterUnstickButton() { return
     * shooterUnstickButton; }
     */

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
        return controller.getRightY();
    }

    @Override
    public double getClimbStickX() {
        return controller.getRightX();
    }

    @Override
    public Trigger getHoodWallButton() {
        return wallButton;
    }

    @Override
    public Trigger getHoodFrontLineButton() {
        return frontLineButton;
    }

    @Override
    public Trigger getHoodBackLineButton() {
        return backLineButton;
    }

    @Override
    public Trigger getHoodTrenchButton() {
        return trenchButton;
    }

    @Override
    public Trigger getManualHoodSwitch() {
        return fakeManualHoodSwitch;
    }

    @Override
    public Trigger getGalacticSearchButton() {
        return galacticSearchButton;
    }

    @Override
    public Trigger getPowerPortAutoButton() {
        return powerPortAutoButton;
    }
}
