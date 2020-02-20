/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.util.TunableNumber;
import com.revrobotics.ControlType;

public class ShooterFlyWheel extends SubsystemBase {

  private static final double defaultRampRate = 2;
  private static final boolean invertFlywheel = true;
  private static final int currentLimit = 30;
  private static final int masterDeviceID = 3;
  private static final int followerDeviceID = 13;
  private static final double MULTIPLIER = 1.5;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANPIDController flywheel_pidController;
  CANEncoder flywheelEncoder;
  public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Double lastRampRate = null; // Force this to be updated once

  private TunableNumber P = new TunableNumber("Shooter FlyWheel PID/P");
  private TunableNumber I = new TunableNumber("Shooter FlyWheel PID/I");
  private TunableNumber D = new TunableNumber("Shooter FlyWheel PID/D");
  private TunableNumber F = new TunableNumber("Shooter FlyWheel PID/F");
  private TunableNumber rampRate = new TunableNumber("Shooter FlyWheel/ramp rate");
  private TunableNumber maxOutput = new TunableNumber("Shooter FlyWheel/Max Output");
  private TunableNumber minOutput = new TunableNumber("Shooter FlyWheel/Min Output");

  /**
   * Creates a new ShooterFlyWheel.
   */
  public ShooterFlyWheel() {
    if (Constants.getRobot() != RobotType.ROBOT_2020 && Constants.getRobot() != RobotType.ROBOT_2020_DRIVE) {
      return;
    }
    flywheelMaster = new CANSparkMax(masterDeviceID, MotorType.kBrushless);
    flywheelFollower = new CANSparkMax(followerDeviceID, MotorType.kBrushless);
    flywheelMaster.restoreFactoryDefaults();
    flywheelFollower.restoreFactoryDefaults();
    flywheelFollower.follow(flywheelMaster, true);

    flywheel_pidController = flywheelMaster.getPIDController();
    flywheelEncoder = flywheelMaster.getEncoder();

    flywheelMaster.setSmartCurrentLimit(currentLimit);
    flywheelFollower.setSmartCurrentLimit(currentLimit);

    flywheelMaster.setInverted(invertFlywheel);

    P.setDefault(0.0012);
    I.setDefault(0);
    D.setDefault(0);
    F.setDefault(0.00019068);
    rampRate.setDefault(defaultRampRate); // Seconds to full power
    maxOutput.setDefault(1);
    minOutput.setDefault(-1);

    // PID coefficients
    kP = P.get();
    kI = I.get();
    kD = D.get();
    kFF = F.get();
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 6000;

    // set PID coefficients
    flywheel_pidController.setP(kP);
    flywheel_pidController.setI(kI);
    flywheel_pidController.setD(kD);
    flywheel_pidController.setFF(kFF);
    flywheel_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Stop by default
    final ShooterFlyWheel subsystem = this;
    this.setDefaultCommand(new Command() {
      @Override
      public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<Subsystem>();
        set.add(subsystem);
        return set;
      }

      @Override
      public void execute() {
        // System.out.println("executing default command");
        subsystem.stop();
      }
    });
    flywheelMaster.burnFlash();
    flywheelFollower.burnFlash();
  }

  @Override
  public void periodic() {
    if (flywheelMaster == null) {
      return;
    }
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = P.get();
    double i = I.get();
    double d = D.get();
    double ff = F.get();
    double max = maxOutput.get();
    double min = minOutput.get();

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      flywheel_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      flywheel_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      flywheel_pidController.setD(d);
      kD = d;
    }
    if ((ff != kFF)) {
      flywheel_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      flywheel_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    double currentRampRate = SmartDashboard.getNumber("Shooter FlyWheel/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      flywheelMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Shooter FlyWheel/speed", getSpeed());
      SmartDashboard.putNumber("Shooter FlyWheel/applied output", flywheelMaster.getAppliedOutput());
    }
  }

  public void stop() {
    if (flywheelMaster == null) {
      return;
    }
    flywheelMaster.stopMotor();
    flywheelFollower.stopMotor();
  }

  public void setShooterRPM(double rpm) {
    if (flywheelMaster == null) {
      return;
    }
    double setpoint = rpm / MULTIPLIER;
    flywheel_pidController.setReference(setpoint, ControlType.kVelocity);
  }

  public void run(double power) {
    if (flywheelMaster == null) {
      return;
    }
    flywheelMaster.set(power);
  }

  public double getSpeed() {
    if (flywheelMaster == null) {
      return 0;
    }
    return flywheelEncoder.getVelocity() * MULTIPLIER;
  }
}
