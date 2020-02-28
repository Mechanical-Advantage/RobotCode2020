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
import frc.robot.oi.IOperatorOI.SetFlyWheelSpeedInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.util.TunableNumber;

import com.revrobotics.ControlType;

public class ShooterFlyWheel extends SubsystemBase {

  private static final double defaultRampRate = 2;
  private static final boolean invertFlywheel = true;
  private static final int currentLimit = 30;
  private static final double MULTIPLIER = 1.5;
  private static final double LEDSlowPulseThreshold = 0.5; // percent of setpoint rpm
  private static final double LEDFastPulseThreshold = 0.9; // percent of setpoint rpm
  private double setpoint = 0;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANPIDController flywheel_pidController;
  CANEncoder flywheelEncoder;
  public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Double lastRampRate = null; // Force this to be updated once
  private OILEDState lastShooterLEDState = OILEDState.OFF;

  private TunableNumber P = new TunableNumber("Shooter FlyWheel PID/P");
  private TunableNumber I = new TunableNumber("Shooter FlyWheel PID/I");
  private TunableNumber D = new TunableNumber("Shooter FlyWheel PID/D");
  private TunableNumber F = new TunableNumber("Shooter FlyWheel PID/F");
  private TunableNumber rampRate = new TunableNumber("Shooter FlyWheel/ramp rate");
  private TunableNumber maxOutput = new TunableNumber("Shooter FlyWheel/Max Output");
  private TunableNumber minOutput = new TunableNumber("Shooter FlyWheel/Min Output");

  private UpdateLEDInterface updateLED;
  private SetFlyWheelSpeedInterface setFlyWheelSpeed;

  /**
   * Creates a new ShooterFlyWheel.
   */
  public ShooterFlyWheel(UpdateLEDInterface updateLED, SetFlyWheelSpeedInterface setFlyWheelSpeed) {
    this.updateLED = updateLED;

    switch (Constants.getRobot()) {
      case ROBOT_2020:
        flywheelMaster = new CANSparkMax(14, MotorType.kBrushless);
        flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
        break;
      case ROBOT_2020_DRIVE:
        flywheelMaster = new CANSparkMax(3, MotorType.kBrushless);
        flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
        break;
      default:
        return;
    }

    this.setFlyWheelSpeed = setFlyWheelSpeed;

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

    setFlyWheelSpeed.set(getSpeed());

    // Update shooter LED
    double targetRpm = setpoint * MULTIPLIER;
    double currentRpm = getSpeed();
    OILEDState shooterLEDState = OILEDState.OFF;
    if (currentRpm > targetRpm * LEDFastPulseThreshold) {
      shooterLEDState = OILEDState.ON;
    } else if (currentRpm > targetRpm * LEDSlowPulseThreshold) {
      shooterLEDState = OILEDState.PULSE_FAST;
    } else if (currentRpm > 0) {
      shooterLEDState = OILEDState.PULSE_SLOW;
    }
    if (shooterLEDState != lastShooterLEDState) {
      updateLED.update(OILED.SHOOTER_SHOOT, shooterLEDState);
      lastShooterLEDState = shooterLEDState;
    }
  }

  public void stop() {
    if (flywheelMaster == null) {
      return;
    }
    flywheelMaster.stopMotor();
    flywheelFollower.stopMotor();
    updateRunningLEDs(false);
  }

  public void setShooterRPM(double rpm) {
    if (flywheelMaster == null) {
      return;
    }
    setpoint = rpm / MULTIPLIER;
    flywheel_pidController.setReference(setpoint, ControlType.kVelocity);
    updateRunningLEDs(rpm != 0);
  }

  public void run(double power) {
    if (flywheelMaster == null) {
      return;
    }
    flywheelMaster.set(power);
    updateRunningLEDs(power != 0);
  }

  public double getSpeed() {
    if (flywheelMaster == null) {
      return 0;
    }
    return flywheelEncoder.getVelocity() * MULTIPLIER;
  }

  private void updateRunningLEDs(boolean running) {
    updateLED.update(OILED.SHOOTER_RUN, running ? OILEDState.ON : OILEDState.OFF);
    updateLED.update(OILED.SHOOTER_STOP, running ? OILEDState.OFF : OILEDState.ON);
  }
}
