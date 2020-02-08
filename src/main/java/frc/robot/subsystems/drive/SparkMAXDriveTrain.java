/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANEncoder;

import frc.robot.Constants;

public class SparkMAXDriveTrain extends DriveTrainBase {
  /**
   * Creates a new SparkMAXDriveTrain.
   */

  private static final int configTimeoutInit = 10;
  private static final int configTimeoutRuntime = 0;

  private SparkMax leftleader;
  private ExternalFollower leftSparkFollower1;
  private ExternalFollower leftSparkFollower2;
  private SparkMax rightleader;
  private ExternalFollower rightSparkFollower1;
  private ExternalFollower rightSparkFollower2;
  private int ticksPerRotation;
  private boolean sixMotorDrive = false;
  public double kP, kI, kD, kF, IntegralZone;

  public SparkMAXDriveTrain(BooleanSupplier driveDisableSwitchAccess, BooleanSupplier openLoopSwitchAccess,
      BooleanSupplier shiftLockSwitchAccess) {
    super(driveDisableSwitchAccess, openLoopSwitchAccess, shiftLockSwitchAccess);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void neutralOutput() {
    // TODO Auto-generated method stub
    leftleader.set(ticksPerRotation * 0);
    rightleader.set(ticksPerRotation * 0);
  }

  @Override
  protected void driveOpenLoopLowLevel(double left, double right) {
    // TODO Auto-generated method stub
    leftleader.set(left);
    rightleader.set(right);
  }

  @Override
  protected void driveClosedLoopLowLevel(double left, double right) {
    // TODO Auto-generated method stub
    leftleader.set(left * ticksPerRotation / 10);
    rightleader.set(right * ticksPerRotation / 10);
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    // TODO Auto-generated method stub
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    leftleader.setNeutralMode(mode);
    leftSparkFollower1.setNeutralMode(mode);
    rightleader.setNeutralMode(mode);
    rightSparkFollower1.setNeutralMode(mode);
    if (sixMotorDrive) {
      leftSparkFollower2.setNeutralMode(mode);
      rightSparkFollower2.setNeutralMode(mode);
    }
  }

  @Override
  public void resetPosition() {
    // TODO Auto-generated method stub
    leftleader.setPosition(0);
    rightleader.setPosition(0);
  }

  @Override
  public double getRotationsLeft() {
    // TODO Auto-generated method stub
    return (double) leftleader.getPosition();
  }

  @Override
  public double getRotationsRight() {
    // TODO Auto-generated method stub
    return (double) rightleader.getPosition();
  }

  @Override
  public double getVelocityRight() {
    // TODO Auto-generated method stub
    return (double) rightleader.getVelocity() / ticksPerRotation * 10;

  }

  @Override
  public double getVelocityLeft() {
    // TODO Auto-generated method stub
    return (double) leftleader.getVelocity() / ticksPerRotation * 10;

  }

  @Override
  public double getCurrent() {
    // TODO Auto-generated method stub
    return (rightleader.getStatorCurrent() + leftleader.getStatorCurrent()) / 2;
  }

  @Override
  protected void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
    // TODO Auto-generated method stub
    leftleader.config_kP(slotIdx, p, configTimeoutRuntime);
    leftleader.config_kI(slotIdx, i, configTimeoutRuntime);
    leftleader.config_kD(slotIdx, d, configTimeoutRuntime);
    leftleader.config_kF(slotIdx, f, configTimeoutRuntime);
    leftleader.config_IntegralZone(slotIdx, iZone, configTimeoutRuntime);
    rightleader.config_kP(slotIdx, p, configTimeoutRuntime);
    rightleader.config_kI(slotIdx, i, configTimeoutRuntime);
    rightleader.config_kD(slotIdx, d, configTimeoutRuntime);
    rightleader.config_kF(slotIdx, f, configTimeoutRuntime);
    rightleader.config_IntegralZone(slotIdx, iZone, configTimeoutRuntime);
  }

  @Override
  public void changeStatusRate(int ms) {
    // TODO Auto-generated method stub
    leftleader.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeoutRuntime);
    rightleader.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeoutRuntime);
  }

  @Override
  public void resetStatusRate() {
    // TODO Auto-generated method stub
    leftleader.setStatusFramePeriod(StatusFrame.Status_1_General, 10, configTimeoutRuntime);
    rightleader.setStatusFramePeriod(StatusFrame.Status_1_General, 10, configTimeoutRuntime);
  }

  @Override
  public void changeSensorRate(int ms) {
    leftleader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, ms, configTimeoutRuntime);
    rightleader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, ms, configTimeoutRuntime);
  }

  @Override
  public void resetSensorRate() {
    leftleader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, configTimeoutRuntime);
    rightleader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, configTimeoutRuntime);
  }

  @Override
  public void changeControlRate(int ms) {
    leftleader.setControlFramePeriod(ControlFrame.Control_3_General, ms);
    rightleader.setControlFramePeriod(ControlFrame.Control_3_General, ms);
  }

  @Override
  public void resetControlRate() {
    leftleader.setControlFramePeriod(ControlFrame.Control_3_General, 10);
    rightleader.setControlFramePeriod(ControlFrame.Control_3_General, 10);
  }

  @Override
  protected void setProfileSlot(int slotIdx) {
    leftleader.selectProfileSlot(slotIdx, 0);
    rightleader.selectProfileSlot(slotIdx, 0);
  }
}
