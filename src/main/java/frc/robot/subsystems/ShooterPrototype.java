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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import com.revrobotics.ControlType;

public class ShooterPrototype extends SubsystemBase {

  private static final double defaultRampRate = 10;
  private static final boolean invertFlywheel = true;
  private static final boolean invertRollers = false;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;
  private CANPIDController flywheel_pidController;
  CANEncoder flywheelEncoder;
  public double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Double lastRampRate = null; // Force this to be updated once

  private TunableNumber P = new TunableNumber("Drive PID/P");
  private TunableNumber I = new TunableNumber("Drive PID/I");
  private TunableNumber D = new TunableNumber("Drive PID/D");
  private TunableNumber F = new TunableNumber("Drive PID/F");
  private TunableNumber setpoint = new TunableNumber("Drive PID/setpoint");

  /**
   * Creates a new ShooterPrototype.
   */
  public ShooterPrototype() {
    SmartDashboard.setDefaultNumber("Shooter Prototype/ramp rate", defaultRampRate); // Seconds to full power
    flywheelMaster = new CANSparkMax(3, MotorType.kBrushless);
    flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
    flywheelFollower.follow(flywheelMaster, true);
    rollerMaster = new CANSparkMax(4, MotorType.kBrushless);
    rollerFollower = new CANSparkMax(11, MotorType.kBrushless);
    rollerFollower.follow(rollerMaster, true);
    flywheelMaster.restoreFactoryDefaults();
    flywheel_pidController = flywheelMaster.getPIDController();
    flywheelEncoder = flywheelMaster.getEncoder();

    // PID coefficients
    // These are copied from SparkMax Example:
    // (https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java)
    kP = 5e-5;
    kI = 1e-6;
    kD = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    flywheel_pidController.setP(kP);
    flywheel_pidController.setI(kI);
    flywheel_pidController.setD(kD);
    flywheel_pidController.setFF(kFF);
    flywheel_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // Stop by default
    final ShooterPrototype subsystem = this;
    this.setDefaultCommand(new Command() {
      @Override
      public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<Subsystem>();
        set.add(subsystem);
        return set;
      }

      @Override
      public void execute() {
        subsystem.runFlywheel(0);
        subsystem.runRollers(0);
      }
    });
  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    P = P.get();
    I = I.get("I Gain", 0);
    D = D.get("D Gain", 0);
    F = F.get("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    double currentRampRate = SmartDashboard.getNumber("Shooter Prototype/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      flywheelMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }
    SmartDashboard.putNumber("Shooter Prototype/speed", getSpeed());

    double setPoint = flywheelMaster.getY() * maxRPM;
    flywheel_pidController.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", flywheelEncoder.getVelocity());
  }

  public void runFlywheel(double power) {
    flywheelMaster.set(power * (invertFlywheel ? -1 : 1));
  }

  public void runRollers(double power) {
    rollerMaster.set(power * (invertRollers ? -1 : 1));
  }

  public double getSpeed() {
    return flywheelEncoder.getVelocity() * 1.5;
  }
}
