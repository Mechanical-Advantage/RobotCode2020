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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frckit.physics.drivetrain.differential.DifferentialDrivetrainDynamics;

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
        leftMaster = new CANSparkMax(3, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightMaster = new CANSparkMax(16, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);
        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        maxVelocityLow = 150;
        kPLow = 0.00015;
        kILow = 0;
        kDLow = 0.0015;
        kIZoneLow = 0;
        leftKsLow = 0.0935;
        leftKvLow = 0.241;
        leftKaLow = 0.0384;
        leftTorquePerVoltLow = (2.6 / 12.0) * 2 * afterEncoderReduction; // NEO torque per volt = (2.6 N*m / 12 V),
                                                                         // times 2 NEOs in each gearbox, times gear
                                                                         // ratio gives torque at wheel.
        rightKsLow = 0.146;
        rightKvLow = 0.241;
        rightKaLow = 0.0331;
        rightTorquePerVoltLow = (2.6 / 12.0) * 2 * afterEncoderReduction;
        massKg = 52.97959;
        moiKgM2 = 6.948569;
        angularDragLow = 0.0;
        wheelDiameter = 3.12207 * 2;
        wheelbaseInches = 24.0;
        trackScrubFactor = 25.934 / wheelbaseInches;
        smartCurrentLimit = 80;
        reverseOutputLeft = true;
        reverseOutputRight = false;
        break;
      case ROBOT_2020_DRIVE:
        leftMaster = new CANSparkMax(2, MotorType.kBrushless);
        leftFollower = new CANSparkMax(12, MotorType.kBrushless);
        rightMaster = new CANSparkMax(14, MotorType.kBrushless);
        rightFollower = new CANSparkMax(15, MotorType.kBrushless);
        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();
        afterEncoderReduction = 1.0 / ((9.0 / 62.0) * (18.0 / 30.0));
        maxVelocityLow = 150;
        kPLow = 0.00015;
        kILow = 0;
        kDLow = 0.0015;
        kIZoneLow = 0;
        leftKsLow = 0.14;
        leftKvLow = 0.2274;
        leftKaLow = 0.0384;
        leftTorquePerVoltLow = Double.POSITIVE_INFINITY;
        rightKsLow = 0.14;
        rightKvLow = 0.2274;
        rightKaLow = 0.0384;
        rightTorquePerVoltLow = Double.POSITIVE_INFINITY;
        massKg = 0;
        moiKgM2 = 0;
        angularDragLow = 0.0;
        wheelDiameter = 6;
        wheelbaseInches = 24.0;
        trackScrubFactor = 24.890470780033485 / wheelbaseInches;
        smartCurrentLimit = 80;
        reverseOutputLeft = true;
        reverseOutputRight = false;
        break;
    }
    dynamicsLow = DifferentialDrivetrainDynamics.fromHybridCharacterization(massKg, moiKgM2, angularDragLow,
        Units.inchesToMeters(wheelDiameter / 2), Units.inchesToMeters(wheelbaseInches) * trackScrubFactor / 2.0,
        leftKsLow, leftKvLow, leftTorquePerVoltLow, rightKsLow, rightKvLow, rightTorquePerVoltLow);
    dynamicsHigh = DifferentialDrivetrainDynamics.fromHybridCharacterization(massKg, moiKgM2, angularDragHigh,
        Units.inchesToMeters(wheelDiameter / 2), Units.inchesToMeters(wheelbaseInches) * trackScrubFactor / 2.0,
        leftKsHigh, leftKvHigh, leftTorquePerVoltHigh, rightKsHigh, rightKvHigh, rightTorquePerVoltHigh);

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
    resetControlRate();
    initialize();
    leftMaster.burnFlash();
    leftFollower.burnFlash();
    rightMaster.burnFlash();
    rightFollower.burnFlash();
    setCANTimeout(configTimeoutRuntime);
  }

  @Override
  public double getGearReduction() {
    return afterEncoderReduction;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Drive L Velocity", getVelocityLeft());
      SmartDashboard.putNumber("Drive R Velocity", getVelocityRight());
      SmartDashboard.putNumber("Drive L Radians", getRotationsLeft() * 2 * Math.PI);
      SmartDashboard.putNumber("Drive R Radians", getRotationsRight() * 2 * Math.PI);
      SmartDashboard.putNumber("Drive L Output", leftMaster.getAppliedOutput() * 100);
      SmartDashboard.putNumber("Drive R Output", rightMaster.getAppliedOutput() * 100);
    }
  }

  @Override
  public void neutralOutput() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
  }

  @Override
  protected void driveOpenLoopLowLevel(double left, double right) {
    leftMaster.setVoltage(left);
    rightMaster.setVoltage(right);
  }

  @Override
  protected void driveClosedLoopLowLevel(double left, double right, double leftVolts, double rightVolts) {
    double leftRpm = left * 60 / (2.0 * Math.PI) * afterEncoderReduction;
    double rightRpm = right * 60 / (2.0 * Math.PI) * afterEncoderReduction;

    leftPidController.setReference(leftRpm, ControlType.kVelocity, currentPidSlot, leftVolts);
    rightPidController.setReference(rightRpm, ControlType.kVelocity, currentPidSlot, rightVolts);
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
  protected void setPID(int slotIdx, double p, double i, double d, int iZone) {
    leftPidController.setP(p, slotIdx);
    leftPidController.setI(i, slotIdx);
    leftPidController.setD(d, slotIdx);
    leftPidController.setFF(0.0, slotIdx);
    leftPidController.setIZone(iZone, slotIdx);
    rightPidController.setP(p, slotIdx);
    rightPidController.setI(i, slotIdx);
    rightPidController.setD(d, slotIdx);
    rightPidController.setFF(0.0, slotIdx);
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
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
    leftMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    rightMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
  }

  @Override
  public void changeControlRate(int ms) {
    leftMaster.setControlFramePeriodMs(ms);
    rightMaster.setControlFramePeriodMs(ms);
  }

  @Override
  public void resetControlRate() {
    leftMaster.setControlFramePeriodMs(0);
    rightMaster.setControlFramePeriodMs(0);
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
