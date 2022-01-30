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
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class Climber extends SubsystemBase {

  private static final int currentLimit = 30;
  private static final IdleMode idleMode = IdleMode.kBrake;
  private static final boolean invertClimber = false;
  private static final int masterDeviceID = 2; // left
  private static final int followerDeviceID = 1; // right
  private static final double followerMultiplier = 1.0;
  private static final int deploySolenoidChannel = 2;
  private static final int PCM = 0;
  private boolean climberEnabled = false;

  CANSparkMax climberMaster;
  CANSparkMax climberFollower;

  private Solenoid climberDeploySolenoid;

  /**
   * Creates a new climber.
   */
  public Climber() {
    if (Constants.getRobot() != RobotType.ROBOT_2020) {
      return;
    }

    climberDeploySolenoid = new Solenoid(PCM, PneumaticsModuleType.CTREPCM, deploySolenoidChannel);
    climberMaster = new CANSparkMax(masterDeviceID, MotorType.kBrushed);
    climberFollower = new CANSparkMax(followerDeviceID, MotorType.kBrushed);

    climberMaster.restoreFactoryDefaults();
    climberFollower.restoreFactoryDefaults();
    climberMaster.setSmartCurrentLimit(currentLimit);
    climberFollower.setSmartCurrentLimit(currentLimit);
    // Default feedback device is hall effect sensor so need to change it
    climberMaster.getEncoder(EncoderType.kNoSensor, 1);
    climberFollower.getEncoder(EncoderType.kNoSensor, 1);
    climberMaster.setIdleMode(idleMode);
    climberFollower.setIdleMode(idleMode);

    climberDeploySolenoid.set(false);

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
        subsystem.run(0, 0);
      }
    });
    climberMaster.burnFlash();
    climberFollower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double left, double right) {
    if (climberMaster == null) {
      return;
    }

    if (climberEnabled) {
      climberMaster.set(left * (invertClimber ? -1 : 1));
      climberFollower.set(right * (invertClimber ? 1 : -1) * followerMultiplier);
    }
  }

  public void deploy() {
    if (climberDeploySolenoid == null) {
      return;
    }
    climberDeploySolenoid.set(true);
    climberEnabled = true;
  }

  public void reset() {
    if (climberDeploySolenoid == null) {
      return;
    }
    climberDeploySolenoid.set(false);
    climberEnabled = false;
    climberMaster.set(0);
    climberFollower.set(0);
  }
}
