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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class Hopper extends SubsystemBase {

  private static final boolean invertLeft = true;
  private static final boolean invertRight = false;
  private static final int leftDeviceID = 5;
  private static final int rightDeviceID = 8;
  private static final int currentLimit = 30;

  CANSparkMax hopperLeft;
  CANSparkMax hopperRight;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    if (Constants.getRobot() != RobotType.ROBOT_2020 && Constants.getRobot() != RobotType.ROBOT_2020_DRIVE) {
      return;
    }
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
    hopperLeft.burnFlash();
    hopperRight.burnFlash();
  }

  @Override
  public void periodic() {
  }

  public void run(double powerLeft, double powerRight) {
    if (hopperLeft == null || hopperRight == null) {
      hopperLeft.set(powerLeft * (invertLeft ? -1 : 1));
      hopperRight.set(powerRight * (invertRight ? -1 : 1));
    }
  }
}
