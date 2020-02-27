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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static final boolean invertIntake = false;
  private static final int solenoidChannel = 3;
  private static final int PCM = 0;

  private static final int currentLimit = 30;

  CANSparkMax intake;

  private Solenoid intakeSolenoid;

  /**
   * Creates a new Intake.
   */
  public Intake() {

    switch (Constants.getRobot()) {
      case ROBOT_2020:
        intake = new CANSparkMax(5, MotorType.kBrushless);
        intakeSolenoid = new Solenoid(PCM, solenoidChannel);
        break;
      case ROBOT_2020_DRIVE:
        // This is temporary for testing
        intake = new CANSparkMax(5, MotorType.kBrushless);
        break;
      default:
        return;
    }

    intake.restoreFactoryDefaults();
    intake.setSmartCurrentLimit(currentLimit);

    // Stop by default
    final Intake subsystem = this;
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
    intake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double power) {
    if (intake == null) {
      return;
    }
    intake.set(power * (invertIntake ? -1 : 1));
  }

  public void extend() {
    if (intakeSolenoid == null) {
      return;
    }
    intakeSolenoid.set(true);
  }

  public void retract() {
    if (intakeSolenoid == null) {
      return;
    }
    intakeSolenoid.set(false);
  }
}
