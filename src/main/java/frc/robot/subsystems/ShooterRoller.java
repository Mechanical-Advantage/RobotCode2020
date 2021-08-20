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
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class ShooterRoller extends SubsystemBase {

  private static final boolean invertRollers = false;
  private static final int currentLimit = 30;
  private static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.15, 0.00103, 0.0000658);

  private TunableNumber kP = new TunableNumber("Roller/P");
  private TunableNumber kI = new TunableNumber("Roller/I");
  private TunableNumber kD = new TunableNumber("Roller/D");

  CANSparkMax rollerMaster;
  CANSparkMax rollerFollower;
  CANEncoder masterEncoder;
  CANEncoder followerEncoder;
  CANPIDController pidController;

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
    rollerFollower.follow(rollerMaster, true);
    rollerMaster.enableVoltageCompensation(12);
    rollerFollower.enableVoltageCompensation(12);
    rollerMaster.setSmartCurrentLimit(currentLimit);
    rollerFollower.setSmartCurrentLimit(currentLimit);

    masterEncoder = rollerMaster.getEncoder();
    followerEncoder = rollerFollower.getEncoder();
    pidController = rollerMaster.getPIDController();

    kP.setDefault(0.00016);
    kI.setDefault(0);
    kD.setDefault(1);

    pidController.setP(kP.get());
    pidController.setI(kI.get());
    pidController.setD(kD.get());
    pidController.setFF(0);

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
    if (Constants.tuningMode) {
      pidController.setP(kP.get());
      pidController.setI(kI.get());
      pidController.setD(kD.get());
      SmartDashboard.putNumber("Roller/VelocityRPM", masterEncoder.getVelocity());
      SmartDashboard.putNumber("Roller/VelocityRPMFollower", followerEncoder.getVelocity());
      SmartDashboard.putNumber("Roller/MasterCurrent", rollerMaster.getOutputCurrent());
      SmartDashboard.putNumber("Roller/FollowerCurrent", rollerFollower.getOutputCurrent());
    }
  }

  public void run(double power) {
    if (rollerMaster == null) {
      return;
    }
    rollerMaster.set(power * (invertRollers ? -1 : 1));
  }

  public void runClosedLoop(double rpm) {
    if (rollerMaster == null) {
      return;
    }
    pidController.setReference(rpm, ControlType.kVelocity, 0, feedForward.calculate(rpm));
  }

  public double getVelocity() {
    if (rollerMaster == null) {
      return 0;
    }
    return masterEncoder.getVelocity();
  }
}
