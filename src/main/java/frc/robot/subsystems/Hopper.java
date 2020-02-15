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

public class Hopper extends SubsystemBase {

  private static final double defaultRampRate = 10;
  private static final boolean invertHopperLeft = true;
  private static final boolean invertHopperRight = false;

  CANSparkMax hopperLeft;
  CANSparkMax hopperRight;

  private Double lastRampRate = null; // Force this to be updated once
  public int currentLimit = 30;
  private double setpointLeft;
  private double setpointRight;
  private int leftDeviceID = 5;
  private int rightDeviceID = 8;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {

    SmartDashboard.setDefaultNumber("Shooter Roller/ramp rate", defaultRampRate); // Seconds to full power
    hopperLeft = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    hopperRight = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
    hopperLeft.restoreFactoryDefaults();
    hopperRight.restoreFactoryDefaults();
    hopperLeft.setSmartCurrentLimit(currentLimit);
    hopperRight.setSmartCurrentLimit(currentLimit);

    // Stop by default
    final Hopper subsystem = this;
    this.setDefaultCommand(new Command() {
      @Override
      public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<Subsystem>();
        set.add(subsystem);
        return set;
      }

      @Override
      public void execute() {
        subsystem.run(0, 0);
      }
    });

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentRampRate = SmartDashboard.getNumber("Hopper/ramp rate", defaultRampRate);
    if (lastRampRate != null && currentRampRate != lastRampRate) {
      hopperLeft.setOpenLoopRampRate(currentRampRate);
      hopperRight.setOpenLoopRampRate(currentRampRate);
      lastRampRate = currentRampRate;
    }

    SmartDashboard.putNumber("Left SetPoint", setpointLeft);
    SmartDashboard.putNumber("Right SetPoint", setpointRight);
  }

  public void run(double powerLeft, double powerRight) {
    hopperLeft.set(powerLeft * (invertHopperLeft ? -1 : 1));
    hopperRight.set(powerRight * (invertHopperRight ? -1 : 1));
  }
}
