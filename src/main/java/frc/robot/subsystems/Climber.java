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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class Climber extends SubsystemBase {

  private static final int currentLimit = 30;
  private static final boolean invertClimber = false;
  private static final int masterDeviceID = 6;
  private static final int followerDeviceID = 10;
  private static final int deploySolenoidChannel = 1;
  private static final int brakeSolenoidChannel = 2;
  private static final int PCM = 0;

  CANSparkMax climberMaster;
  CANSparkMax climberFollower;

  private Solenoid climberDeploySolenoid;
  private Solenoid climberBrakeSolenoid;

  /**
   * Creates a new climber.
   */
  public Climber() {
    if (Constants.getRobot() != RobotType.ROBOT_2020 && Constants.getRobot() != RobotType.ROBOT_2020_DRIVE) {
      return;
    }

    climberDeploySolenoid = new Solenoid(deploySolenoidChannel, PCM);
    climberBrakeSolenoid = new Solenoid(brakeSolenoidChannel, PCM);
    climberMaster = new CANSparkMax(masterDeviceID, MotorType.kBrushless);
    climberFollower = new CANSparkMax(followerDeviceID, MotorType.kBrushless);

    climberMaster.restoreFactoryDefaults();
    climberFollower.restoreFactoryDefaults();
    climberFollower.follow(climberMaster, true);
    climberMaster.setSmartCurrentLimit(currentLimit);
    climberFollower.setSmartCurrentLimit(currentLimit);

    climberDeploySolenoid.set(false);
    climberBrakeSolenoid.set(true);

    // Stop by default
    final Climber subsystem = this;
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
    climberMaster.burnFlash();
    climberFollower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double power) {
    if (climberMaster == null) {
      return;
    }
    climberMaster.set(power * (invertClimber ? -1 : 1));
  }

  public void deploy() {
    if (climberDeploySolenoid == null) {
      return;
    }
    climberDeploySolenoid.set(true);
  }

  public void brake() {
    if (climberDeploySolenoid == null) {
      return;
    }
    climberBrakeSolenoid.set(false);
    // from 2018 code format:
    // climberBrakeSolenoid.set(Value.kForward);
  }
}
