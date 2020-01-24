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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPrototype extends SubsystemBase {

  private static final double defaultRampRate = 10;
  private static final boolean invertFlywheel = true;
  private static final boolean invertRollers = false;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;
  CANEncoder flywheelEncoder;

  private Double lastRampRate = null; // Force this to be updated once

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
    flywheelEncoder = flywheelMaster.getEncoder();

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
    double currentRampRate = SmartDashboard.getNumber("Shooter Prototype/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      flywheelMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }
    SmartDashboard.putNumber("Shooter Prototype/speed", getSpeed());
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
