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

public class ShooterRoller extends SubsystemBase {

  private static final boolean invertRollers = false;
  private static final int currentLimit = 30;
  private static final int masterDeviceID = 4;
  private static final int followerDeviceID = 11;

  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;

  /**
   * Creates a new ShooterRoller.
   */
  public ShooterRoller() {
    if (Constants.getRobot() != RobotType.ROBOT_2020 && Constants.getRobot() != RobotType.ROBOT_2020_DRIVE) {
      return;
    }
    rollerMaster = new CANSparkMax(masterDeviceID, MotorType.kBrushless);
    rollerFollower = new CANSparkMax(followerDeviceID, MotorType.kBrushless);
    rollerMaster.restoreFactoryDefaults();
    rollerFollower.restoreFactoryDefaults();
    rollerFollower.follow(rollerMaster, true);
    rollerMaster.setSmartCurrentLimit(currentLimit);
    rollerFollower.setSmartCurrentLimit(currentLimit);

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
    rollerMaster.burnFlash();
    rollerFollower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double power) {
    if (rollerMaster == null) {
      return;
    }
    rollerMaster.set(power * (invertRollers ? -1 : 1));
  }
}
