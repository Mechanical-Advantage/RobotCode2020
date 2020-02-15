/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;

public class SparkMAXDriveTrain extends DriveTrainBase {

  private static final int configTimeoutInit = 10;
  private static final int configTimeoutRuntime = 0;

  private CANSparkMax leftMaster;
  private CANSparkMax leftFollower;
  private CANSparkMax rightMaster;
  private CANSparkMax rightFollower;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private CANPIDController leftPidController;
  private CANPIDController rightPidController;
  private double afterEncoderReduction = 1;
  private int smartCurrentLimit = 80;
  private boolean reverseOutputLeft;
  private boolean reverseOutputRight;

  private int currentPidSlot;

  /**
   * Creates a new SparkMAXDriveTrain.
   */
  @SuppressWarnings("incomplete-switch")
  public SparkMAXDriveTrain(BooleanSupplier driveDisableSwitchAccess, BooleanSupplier openLoopSwitchAccess,
      BooleanSupplier shiftLockSwitchAccess) {
    super(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
    switch (Constants.getRobot()) {
    case ROBOT_2020:
    case ROBOT_2020_DRIVE:
      leftMaster = new CANSparkMax(2, MotorType.kBrushless);
      leftFollower = new CANSparkMax(12, MotorType.kBrushless);
      rightMaster = new CANSparkMax(14, MotorType.kBrushless);
      rightFollower = new CANSparkMax(15, MotorType.kBrushless);
      leftEncoder = leftMaster.getEncoder();
      rightEncoder = rightMaster.getEncoder();
      minVelocityLow = 0;
      maxVelocityLow = 1;
      kPLow = 0;
      kILow = 0;
      kDLow = 0;
      kFLow = 0;
      kIZoneLow = 0;
      wheelDiameter = 6;
      smartCurrentLimit = 80;
      reverseOutputLeft = true;
      reverseOutputRight = false;
      afterEncoderReduction = 11.5;
    }
    setCANTimeout(configTimeoutInit);
    leftMaster.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftPidController = leftMaster.getPIDController();
    rightPidController = rightMaster.getPIDController();
    leftMaster.setSmartCurrentLimit(smartCurrentLimit);
    leftFollower.setSmartCurrentLimit(smartCurrentLimit);
    rightMaster.setSmartCurrentLimit(smartCurrentLimit);
    rightFollower.setSmartCurrentLimit(smartCurrentLimit);
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    leftMaster.setInverted(reverseOutputLeft);
    rightMaster.setInverted(reverseOutputRight);
    resetSensorRate(); // SPARK MAX default doesn't match ours
    initialize();
    leftMaster.burnFlash();
    leftFollower.burnFlash();
    rightMaster.burnFlash();
    rightFollower.burnFlash();
    setCANTimeout(configTimeoutRuntime);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void neutralOutput() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
  }

  @Override
  protected void driveOpenLoopLowLevel(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
  }

  @Override
  protected void driveClosedLoopLowLevel(double left, double right) {
    leftPidController.setReference(left * 60 * afterEncoderReduction, ControlType.kVelocity, currentPidSlot);
    rightPidController.setReference(right * 60 * afterEncoderReduction, ControlType.kVelocity, currentPidSlot);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    leftMaster.setIdleMode(mode);
    leftFollower.setIdleMode(mode);
    rightMaster.setIdleMode(mode);
    rightFollower.setIdleMode(mode);
  }

  @Override
  public void resetPosition() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public double getRotationsLeft() {
    return leftEncoder.getPosition() / afterEncoderReduction;
  }

  @Override
  public double getRotationsRight() {
    return rightEncoder.getPosition() / afterEncoderReduction;
  }

  @Override
  public double getRPSRight() {
    return rightEncoder.getVelocity() / afterEncoderReduction / 60;

  }

  @Override
  public double getRPSLeft() {
    return leftEncoder.getVelocity() / afterEncoderReduction / 60;
  }

  @Override
  public double getCurrent() {
    return (rightMaster.getOutputCurrent() + leftMaster.getOutputCurrent() + rightFollower.getOutputCurrent()
        + leftFollower.getOutputCurrent()) / 4;
  }

  @Override
  protected void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
    leftPidController.setP(p, slotIdx);
    leftPidController.setI(i, slotIdx);
    leftPidController.setD(d, slotIdx);
    leftPidController.setFF(f, slotIdx);
    leftPidController.setIZone(iZone, slotIdx);
    rightPidController.setP(p, slotIdx);
    rightPidController.setI(i, slotIdx);
    rightPidController.setD(d, slotIdx);
    rightPidController.setFF(f, slotIdx);
    rightPidController.setIZone(iZone, slotIdx);
  }

  @Override
  public void changeStatusRate(int ms) {
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ms);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ms);
  }

  @Override
  public void resetStatusRate() {
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
  }

  @Override
  public void changeSensorRate(int ms) {
    // Status 1 has velocity among other things
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
    // Status 2 has position
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
  }

  @Override
  public void resetSensorRate() {
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    // The SPARK MAX defaults to 50ms for status 2 which is very slow for a drive
    // train so use 20ms instead here
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
  }

  @Override
  public void changeControlRate(int ms) {
    leftMaster.setControlFramePeriodMs(ms);
    rightMaster.setControlFramePeriodMs(ms);
  }

  @Override
  public void resetControlRate() {
    leftMaster.setControlFramePeriodMs(10);
    rightMaster.setControlFramePeriodMs(10);
  }

  @Override
  protected void setProfileSlot(int slotIdx) {
    currentPidSlot = slotIdx;
  }

  private void setCANTimeout(int ms) {
    leftMaster.setCANTimeout(ms);
    leftFollower.setCANTimeout(ms);
    rightMaster.setCANTimeout(ms);
    rightFollower.setCANTimeout(ms);
  }
}
