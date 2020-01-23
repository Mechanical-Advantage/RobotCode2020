/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPrototype extends SubsystemBase {

  private static final double defaultRampRate = 10;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;

  private Double lastRampRate = null; // Force this to be updated once

  /**
   * Creates a new ShooterPrototype.
   */
  public ShooterPrototype() {
    SmartDashboard.setDefaultNumber("Shooter Prototype/ramp rate", defaultRampRate); // Seconds to full power
    flywheelMaster = new CANSparkMax(3, MotorType.kBrushless);
    flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
    flywheelFollower.follow(flywheelMaster, true);
    rollerMaster = new CANSparkMax(5, MotorType.kBrushless);
    rollerFollower = new CANSparkMax(11, MotorType.kBrushless);
    rollerFollower.follow(rollerMaster, true);

    // Stop by default
    this.setDefaultCommand(new InstantCommand(() -> {
      this.runFlywheel(0);
      this.runRollers(0);
    }, this));
  }

  @Override
  public void periodic() {
    double currentRampRate = SmartDashboard.getNumber("Shooter Prototype/ramp rate", defaultRampRate);
    if (currentRampRate != lastRampRate) {
      flywheelMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }
  }

  public void runFlywheel(double power) {
    flywheelMaster.set(power);
  }

  public void runRollers(double power) {
    rollerMaster.set(power);
  }
}
