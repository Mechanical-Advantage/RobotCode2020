/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class CTREDriveTrain extends DriveTrainBase {

  private static final int configTimeoutInit = 10;
  private static final int configTimeoutRuntime = 0;

  private TalonSRX leftMaster;
  private BaseMotorController leftFollower1;
  private BaseMotorController leftFollower2;
  private TalonSRX rightMaster;
  private BaseMotorController rightFollower1;
  private BaseMotorController rightFollower2;
  private int ticksPerRotation;
  private boolean sixMotorDrive = false;

  /**
   * Creates a new CTREDriveTrain.
   */
  @SuppressWarnings("incomplete-switch") // Not all robots need to be supported
  public CTREDriveTrain(BooleanSupplier driveDisableSwitchAccess, BooleanSupplier openLoopSwitchAccess,
      BooleanSupplier shiftLockSwitchAccess) {
    super(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
    FeedbackDevice encoderType = FeedbackDevice.None;
    boolean reverseOutputLeft = false;
    boolean reverseOutputRight = false;
    boolean reverseSensorLeft = false;
    boolean reverseSensorRight = false;
    switch (Constants.getRobot()) {
    case NOTBOT:
      leftMaster = new TalonSRX(1);
      leftFollower1 = new TalonSRX(2);
      rightMaster = new TalonSRX(3);
      rightFollower1 = new TalonSRX(4);
      maxVelocityLow = 122.282131;
      minVelocityLow = 5.14872;
      encoderType = FeedbackDevice.QuadEncoder;
      ticksPerRotation = 1440;
      wheelDiameter = 5.9000000002; // 6
      reverseSensorRight = false;
      reverseSensorLeft = false;
      reverseOutputLeft = false;
      reverseOutputRight = true;
      kPLow = 2;
      kILow = 0;
      kDLow = 40;
      kFLow = 1.07;
      kIZoneLow = 0;
      break;
    case ORIGINAL_ROBOT_2018:
      leftMaster = new TalonSRX(12);
      leftFollower1 = new TalonSRX(13);
      rightMaster = new TalonSRX(2);
      rightFollower1 = new TalonSRX(0);
      leftGearSolenoid1 = 0;
      leftGearSolenoid2 = 1;
      leftGearPCM = 1;
      rightGearSolenoid1 = 2;
      rightGearSolenoid2 = 3;
      rightGearPCM = 0;
      maxVelocityLow = 106;
      maxVelocityHigh = 230;
      minVelocityLow = 3.25971;
      minVelocityHigh = 13.038837;
      dualGear = true;
      encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
      ticksPerRotation = 4096;
      wheelDiameter = 4.25;
      reverseSensorRight = false;
      reverseSensorLeft = false;
      reverseOutputLeft = true;
      reverseOutputRight = false;
      kPLow = 0.5;
      kILow = 0.003;
      kIZoneLow = 300;
      kDLow = 30;
      kFLow = 0.3145756458;
      kPHigh = 0.8;
      kIHigh = 0;
      kDHigh = 10;
      kFHigh = 0.1449829932;
      break;
    case ROBOT_2019:
      leftMaster = new TalonSRX(12);
      leftFollower1 = new VictorSPX(14);
      leftFollower2 = new VictorSPX(13);
      rightMaster = new TalonSRX(3);
      rightFollower1 = new VictorSPX(2);
      rightFollower2 = new VictorSPX(1);
      ptoSolenoid1 = 3;
      ptoSolenoid2 = 2;
      ptoPCM = 1;
      maxVelocityLow = 170.566392;
      minVelocityLow = 7.4622796;
      sixMotorDrive = true;
      encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
      ticksPerRotation = 4096;
      wheelDiameter = 4.633; // Testing of DriveDistanceOnHeading suggests this may not be right
      reverseSensorRight = true;
      reverseSensorLeft = true;
      reverseOutputLeft = true;
      reverseOutputRight = false;
      kPLow = 0.8;
      kILow = 0;
      kIZoneLow = 0;
      kDLow = 30;
      kFLow = 0.23; // Calculated 0.213125
      hasPTO = true;
      break;
    case REBOT:
      leftMaster = new TalonSRX(12);
      leftFollower1 = new VictorSPX(13);
      leftFollower2 = new VictorSPX(14);
      rightMaster = new TalonSRX(1);
      rightFollower1 = new VictorSPX(2);
      rightFollower2 = new VictorSPX(3);
      maxVelocityLow = 125.94;
      minVelocityLow = 5.18;
      sixMotorDrive = true;
      encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
      ticksPerRotation = 4096;
      wheelDiameter = 5; // This is just a best guess, make sure to measure before tuning
      reverseSensorRight = true;
      reverseSensorLeft = true;
      reverseOutputLeft = false;
      reverseOutputRight = true;
      kPLow = 0.6;
      kILow = 0.0007;
      kDLow = 6;
      kFLow = 0.2842;
      kIZoneLow = 4096 * 50 / 600;
      break;
    }
    leftMaster.configFactoryDefault(configTimeoutInit);
    leftFollower1.configFactoryDefault(configTimeoutInit);
    leftMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeoutInit);
    leftMaster.setInverted(reverseOutputLeft);
    leftFollower1.setInverted(reverseOutputLeft);
    leftMaster.setSensorPhase(reverseSensorLeft);
    leftFollower1.follow(leftMaster);
    rightMaster.configFactoryDefault(configTimeoutInit);
    rightFollower1.configFactoryDefault(configTimeoutInit);
    rightMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeoutInit);
    rightMaster.setInverted(reverseOutputRight);
    rightFollower1.setInverted(reverseOutputRight);
    rightMaster.setSensorPhase(reverseSensorRight);
    rightFollower1.follow(rightMaster);
    if (sixMotorDrive) {
      leftFollower2.configFactoryDefault(configTimeoutInit);
      leftFollower2.setInverted(reverseOutputLeft);
      leftFollower2.follow(leftMaster);
      rightFollower2.configFactoryDefault(configTimeoutInit);
      rightFollower2.setInverted(reverseOutputRight);
      rightFollower2.follow(rightMaster);
    }
    initialize();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void neutralOutput() {
    leftMaster.neutralOutput();
    rightMaster.neutralOutput();
  }

  @Override
  protected void driveOpenLoopLowLevel(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, right);
  }

  @Override
  protected void driveClosedLoopLowLevel(double left, double right) {
    leftMaster.set(ControlMode.Velocity, left * ticksPerRotation / 10);
    rightMaster.set(ControlMode.Velocity, right * ticksPerRotation / 10);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    leftMaster.setNeutralMode(mode);
    leftFollower1.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    rightFollower1.setNeutralMode(mode);
    if (sixMotorDrive) {
      leftFollower2.setNeutralMode(mode);
      rightFollower2.setNeutralMode(mode);
    }
  }

  @Override
  public void resetPosition() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  @Override
  public double getRotationsLeft() {
    return (double) leftMaster.getSelectedSensorPosition() / ticksPerRotation;
  }

  @Override
  public double getRotationsRight() {
    return (double) rightMaster.getSelectedSensorPosition() / ticksPerRotation;
  }

  @Override
  public double getVelocityRight() {
    return (double) rightMaster.getSelectedSensorVelocity() / ticksPerRotation * 10;
  }

  @Override
  public double getVelocityLeft() {
    return (double) leftMaster.getSelectedSensorVelocity() / ticksPerRotation * 10;
  }

  @Override
  public double getCurrent() {
    return (rightMaster.getStatorCurrent() + leftMaster.getStatorCurrent()) / 2;
  }

  @Override
  protected void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
    leftMaster.config_kP(slotIdx, p, configTimeoutRuntime);
    leftMaster.config_kI(slotIdx, i, configTimeoutRuntime);
    leftMaster.config_kD(slotIdx, d, configTimeoutRuntime);
    leftMaster.config_kF(slotIdx, f, configTimeoutRuntime);
    leftMaster.config_IntegralZone(slotIdx, iZone, configTimeoutRuntime);
    rightMaster.config_kP(slotIdx, p, configTimeoutRuntime);
    rightMaster.config_kI(slotIdx, i, configTimeoutRuntime);
    rightMaster.config_kD(slotIdx, d, configTimeoutRuntime);
    rightMaster.config_kF(slotIdx, f, configTimeoutRuntime);
    rightMaster.config_IntegralZone(slotIdx, iZone, configTimeoutRuntime);
  }

  @Override
  public void changeStatusRate(int ms) {
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeoutRuntime);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeoutRuntime);
  }

  @Override
  public void resetSensorRate() {
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20, configTimeoutRuntime);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 20, configTimeoutRuntime);
  }

  @Override
  public void changeControlRate(int ms) {
    leftMaster.setControlFramePeriod(ControlFrame.Control_3_General, ms);
    rightMaster.setControlFramePeriod(ControlFrame.Control_3_General, ms);
  }

  @Override
  public void resetControlRate() {
    leftMaster.setControlFramePeriod(ControlFrame.Control_3_General, 10);
    rightMaster.setControlFramePeriod(ControlFrame.Control_3_General, 10);
  }

  @Override
  protected void setProfileSlot(int slotIdx) {
    leftMaster.selectProfileSlot(slotIdx, 0);
    rightMaster.selectProfileSlot(slotIdx, 0);
  }
}
