/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPrototype extends SubsystemBase {

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;

  /**
   * Creates a new ShooterPrototype.
   */
  public ShooterPrototype() {
    flywheelMaster = new CANSparkMax(3, MotorType.kBrushless);
    flywheelMaster.setOpenLoopRampRate(5); // Seconds to full power
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
    // This method will be called once per scheduler run
  }

  public void runFlywheel(double power) {
    flywheelMaster.set(power);
  }

  public void runRollers(double power) {
    rollerMaster.set(power);
  }
}
