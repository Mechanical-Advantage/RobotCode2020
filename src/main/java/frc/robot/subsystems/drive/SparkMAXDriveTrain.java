/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;

public class SparkMAXDriveTrain extends DriveTrainBase {
  /**
   * Creates a new SparkMAXDriveTrain.
   */
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

  }

  @Override
  protected void driveOpenLoopLowLevel(double left, double right) {
    // TODO Auto-generated method stub

  }

  @Override
  protected void driveClosedLoopLowLevel(double left, double right) {
    // TODO Auto-generated method stub

  }

  @Override
  public void enableBrakeMode(boolean enable) {
    // TODO Auto-generated method stub

  }

  @Override
  public void resetPosition() {
    // TODO Auto-generated method stub

  }

  @Override
  public double getRotationsLeft() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getRotationsRight() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocityRight() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocityLeft() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getCurrent() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  protected void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
    // TODO Auto-generated method stub

  }

  @Override
  public void changeStatusRate(int ms) {
    // TODO Auto-generated method stub

  }

  @Override
  public void resetStatusRate() {
    // TODO Auto-generated method stub

  }

  @Override
  public void changeSensorRate(int ms) {
    // TODO Auto-generated method stub

  }

  @Override
  public void resetSensorRate() {
    // TODO Auto-generated method stub

  }

  @Override
  public void changeControlRate(int ms) {
    // TODO Auto-generated method stub

  }

  @Override
  public void resetControlRate() {
    // TODO Auto-generated method stub

  }

  @Override
  protected void setProfileSlot(int slotIdx) {
    // TODO Auto-generated method stub

  }
}
