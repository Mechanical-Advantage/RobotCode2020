/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterRoller extends SubsystemBase {

  private static final double defaultRampRate = 10;
  private static final boolean invertRollers = false;

  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;

  private Double lastRampRate = null; // Force this to be updated once
  private double setpoint;
  public int currentLimit = 30;
  private int masterDeviceID = 4;
  private int followerDeviceID = 11;

  /**
   * Creates a new ShooterRoller.
   */
  public ShooterRoller() {
    SmartDashboard.setDefaultNumber("Shooter Roller/ramp rate", defaultRampRate); // Seconds to full power
    rollerMaster = new CANSparkMax(masterDeviceID, MotorType.kBrushless);
    rollerFollower = new CANSparkMax(followerDeviceID, MotorType.kBrushless);
    rollerMaster.restoreFactoryDefaults();
    rollerFollower.restoreFactoryDefaults();
    rollerFollower.follow(rollerMaster, true);
    rollerMaster.setSmartCurrentLimit(currentLimit);

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
        subsystem.run(0);
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentRampRate = SmartDashboard.getNumber("Shooter Roller/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      rollerMaster.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }

    SmartDashboard.putNumber("SetPoint", setpoint);
  }

  public void run(double power) {
    rollerMaster.set(power * (invertRollers ? -1 : 1));
  }
}
