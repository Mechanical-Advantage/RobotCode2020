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
import frc.robot.util.TunableNumber;
import com.revrobotics.ControlType;

public class ShooterRoller extends SubsystemBase {

  private static final double defaultRampRate = 10;
  private static final boolean invertRollers = false;

  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;
  CANPIDController roller_pidController;
  CANEncoder flywheelEncoder;
  public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Double lastRampRate = null; // Force this to be updated once

  private TunableNumber P = new TunableNumber("Shooter Roller PID/P");
  private TunableNumber I = new TunableNumber("Shooter Roller PID/I");
  private TunableNumber D = new TunableNumber("Shooter Roller PID/D");
  private TunableNumber F = new TunableNumber("Shooter Roller PID/F");
  private double setpoint;
  private double multiplier;

  /**
   * Creates a new ShooterRoller.
   */
  public ShooterRoller() {
    SmartDashboard.setDefaultNumber("Shooter Roller/ramp rate", defaultRampRate); // Seconds to full power
    rollerMaster = new CANSparkMax(4, MotorType.kBrushless);
    rollerFollower = new CANSparkMax(11, MotorType.kBrushless);
    rollerFollower.follow(rollerMaster, true);
    rollerMaster.restoreFactoryDefaults();
    roller_pidController = rollerMaster.getPIDController();

    P.setDefault(0);
    I.setDefault(0);
    D.setDefault(0);
    F.setDefault(0);

    // PID coefficients
    kP = P.get();
    kI = I.get();
    kD = D.get();
    kFF = F.get();
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    roller_pidController.setP(kP);
    roller_pidController.setI(kI);
    roller_pidController.setD(kD);
    roller_pidController.setFF(kFF);
    roller_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // Stop by default
    final ShooterRoller subsystem = this;
    this.setDefaultCommand(new Command() {
      @Override
      public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<Subsystem>();
        set.add(subsystem);
        return set;
      }

      @Override
      public void execute() {
        subsystem.runRollers(0);
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = P.get();
    double i = I.get();
    double d = D.get();
    double ff = F.get();
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      roller_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      roller_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      roller_pidController.setD(d);
      kD = d;
    }
    if ((ff != kFF)) {
      roller_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      roller_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    double currentRampRate = SmartDashboard.getNumber("Shooter Roller/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      rollerMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }
    SmartDashboard.putNumber("Shooter Roller/speed", getSpeed());

    roller_pidController.setReference(setpoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setpoint);
  }

  public void setShooterRPM(double rpm) {
    setpoint = rpm * multiplier;
    // %
  }

  public void runRollers(double power) {
    rollerMaster.set(power * (invertRollers ? -1 : 1));
  }

  public double getSpeed() {
    return flywheelEncoder.getVelocity() * 1.5;
  }
}
