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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ShooterRoller extends SubsystemBase {

  private static final boolean invertRollers = false;
  private static final int currentLimit = 30;

  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;
  CANEncoder encoder;

  /**
   * Creates a new ShooterRoller.
   */
  public ShooterRoller() {
    switch (Constants.getRobot()) {
      case ROBOT_2020:
      case ROBOT_2020_DRIVE:
        rollerMaster = new CANSparkMax(8, MotorType.kBrushless);
        rollerFollower = new CANSparkMax(11, MotorType.kBrushless);
        break;
      default:
        return;
    }

    rollerMaster.restoreFactoryDefaults();
    rollerFollower.restoreFactoryDefaults();
    rollerMaster.enableVoltageCompensation(12);
    rollerFollower.enableVoltageCompensation(12);
    rollerFollower.follow(rollerMaster, true);
    rollerMaster.setSmartCurrentLimit(currentLimit);
    rollerFollower.setSmartCurrentLimit(currentLimit);

    encoder = rollerMaster.getEncoder();

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

  public double getVoltage() {
    if (rollerMaster == null) {
      return 0;
    }
    return rollerMaster.getAppliedOutput() * 12;
  }

  public double getPosition() {
    if (rollerMaster == null) {
      return 0;
    }
    return encoder.getPosition();
  }

  public double getVelocity() {
    if (rollerMaster == null) {
      return 0;
    }
    return encoder.getVelocity();
  }
}
