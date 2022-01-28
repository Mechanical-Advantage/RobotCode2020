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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;

public class Intake extends SubsystemBase {

  private static final boolean invertIntake = true;
  private static final int solenoidChannel = 3;
  private static final int PCM = 0;

  private static final int currentLimit = 30;

  private final UpdateLEDInterface updateLED;

  CANSparkMax intake;

  private Solenoid intakeSolenoid;

  /**
   * Creates a new Intake.
   */
  public Intake(UpdateLEDInterface updateLED) {
    this.updateLED = updateLED;

    switch (Constants.getRobot()) {
      case ROBOT_2020:
        intake = new CANSparkMax(5, MotorType.kBrushed);
        intake.getEncoder(EncoderType.kNoSensor, 0);
        intakeSolenoid = new Solenoid(PCM, PneumaticsModuleType.CTREPCM, solenoidChannel);
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
    setRunLEDs(power);
  }

  public void extend() {
    if (intakeSolenoid == null) {
      return;
    }
    intakeSolenoid.set(true);
    setExtendRetractLEDs(true);
  }

  public void retract() {
    if (intakeSolenoid == null) {
      return;
    }
    intakeSolenoid.set(false);
    setExtendRetractLEDs(false);
  }

  public boolean isExtended() {
    if (intakeSolenoid == null) {
      return false;
    }
    return intakeSolenoid.get();
  }

  private void setExtendRetractLEDs(boolean extended) {
    updateLED.update(OILED.INTAKE_EXTEND, extended ? OILEDState.MED : OILEDState.OFF);
    updateLED.update(OILED.INTAKE_RETRACT, extended ? OILEDState.OFF : OILEDState.MED);
  }

  private void setRunLEDs(double power) {
    if (power == 0) {
      updateLED.update(OILED.INTAKE_FORWARD, OILEDState.OFF);
      updateLED.update(OILED.INTAKE_BACKWARD, OILEDState.OFF);
    } else if (power > 0) {
      updateLED.update(OILED.INTAKE_FORWARD, OILEDState.ON);
    } else if (power < 0) {
      updateLED.update(OILED.INTAKE_BACKWARD, OILEDState.ON);
    }
  }

  public void updateLEDs() {
    if (intakeSolenoid == null || intake == null) {
      return;
    }
    setExtendRetractLEDs(intakeSolenoid.get());
    setRunLEDs(intake.getAppliedOutput());
  }
}
