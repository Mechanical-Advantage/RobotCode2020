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
  private static final int currentLimit = 30;

  CANSparkMax hopperLeft;
  CANSparkMax hopperRight;

  /**
   * Creates a new Hopper.
   */
  public Hopper() {
    switch (Constants.getRobot()) {
    case ROBOT_2020:
      hopperLeft = new CANSparkMax(7, MotorType.kBrushless);
      hopperRight = new CANSparkMax(4, MotorType.kBrushless);
      break;
    case ROBOT_2020_DRIVE:
      hopperLeft = new CANSparkMax(5, MotorType.kBrushless);
      hopperRight = new CANSparkMax(8, MotorType.kBrushless);
      break;
    default:
      return;
    }

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
      return;
    }
    hopperLeft.set(powerLeft * (invertLeft ? -1 : 1));
    hopperRight.set(powerRight * (invertRight ? -1 : 1));
  }
}
