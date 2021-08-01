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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.SetFlyWheelSpeedInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.oi.IOperatorOI.OILED;
import frc.robot.oi.IOperatorOI.OILEDState;
import frc.robot.util.TunableNumber;
import frc.robot.util.VelocityProfiler;

import com.revrobotics.ControlType;

public class ShooterFlyWheel extends SubsystemBase {

  private static final double defaultOpenLoopRampRate = 2;
  private static final boolean invertFlywheel = true;
  private static final int currentLimit = 30;
  private static final double MULTIPLIER = 1.5;
  private static final double LEDSlowPulseThreshold = 0.5; // percent of setpoint rpm
  private static final double atSetpointThreshold = 0.95; // percent of setpoint rpm
  private static final double safeFeedThreshold = 2500; // min rpm to feed balls where they won't get stuck
  private static final double accurateMinTheshold = 0.97;
  private static final double accurateMaxTheshold = 1.03;
  private static final double accurateInTime = 2;
  private static final double accurateFeedGrace = 0.1;
  private boolean accurateReady = false;
  private Timer accurateInTimer = new Timer();
  private Timer accurateGraceTimer = new Timer();
  private double setpoint = 0;

  CANSparkMax flywheelMaster;
  CANSparkMax flywheelFollower;
  CANPIDController flywheel_pidController;
  CANEncoder flywheelEncoder;
  public double kP, kI, kD, kMaxOutput, kMinOutput, maxRPM;
  private SimpleMotorFeedforward feedForwardModel;
  private boolean openLoopControl = true;

  private Double lastOpenLoopRampRate = null; // Force this to be updated once
  private VelocityProfiler closedLoopVelocityProfiler = new VelocityProfiler(6000);
  private OILEDState lastShooterLEDState = OILEDState.OFF;

  private TunableNumber P = new TunableNumber("Shooter FlyWheel PID/P");
  private TunableNumber I = new TunableNumber("Shooter FlyWheel PID/I");
  private TunableNumber D = new TunableNumber("Shooter FlyWheel PID/D");
  private TunableNumber openLoopRampRate = new TunableNumber("Shooter FlyWheel/Open Loop Ramp Rate");
  private TunableNumber maxOutput = new TunableNumber("Shooter FlyWheel/Max Output");
  private TunableNumber minOutput = new TunableNumber("Shooter FlyWheel/Min Output");

  private UpdateLEDInterface updateLED;
  private SetFlyWheelSpeedInterface setFlyWheelSpeed;

  /**
   * Creates a new ShooterFlyWheel.
   */
  public ShooterFlyWheel(UpdateLEDInterface updateLED, SetFlyWheelSpeedInterface setFlyWheelSpeed) {
    this.updateLED = updateLED;

    switch (Constants.getRobot()) {
      case ROBOT_2020:
        flywheelMaster = new CANSparkMax(14, MotorType.kBrushless);
        flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
        feedForwardModel = new SimpleMotorFeedforward(0.133, 0.00142, 0.000489);
        break;
      case ROBOT_2020_DRIVE:
        flywheelMaster = new CANSparkMax(3, MotorType.kBrushless);
        flywheelFollower = new CANSparkMax(13, MotorType.kBrushless);
        feedForwardModel = new SimpleMotorFeedforward(0, 0, 0);
        break;
      default:
        return;
    }

    accurateInTimer.reset();
    accurateInTimer.start();
    accurateGraceTimer.reset();
    accurateGraceTimer.start();

    this.setFlyWheelSpeed = setFlyWheelSpeed;

    flywheelMaster.restoreFactoryDefaults();
    flywheelFollower.restoreFactoryDefaults();
    flywheelFollower.follow(flywheelMaster, true);

    flywheel_pidController = flywheelMaster.getPIDController();
    flywheelEncoder = flywheelMaster.getEncoder();

    flywheelMaster.setSmartCurrentLimit(currentLimit);
    flywheelFollower.setSmartCurrentLimit(currentLimit);

    flywheelMaster.setInverted(invertFlywheel);

    P.setDefault(0.0005);
    I.setDefault(0);
    D.setDefault(0.0015);
    openLoopRampRate.setDefault(defaultOpenLoopRampRate); // Seconds to full power
    maxOutput.setDefault(1);
    minOutput.setDefault(-1);

    // PID coefficients
    kP = P.get();
    kI = I.get();
    kD = D.get();
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 6000;

    // set PID coefficients
    flywheel_pidController.setP(kP);
    flywheel_pidController.setI(kI);
    flywheel_pidController.setD(kD);
    flywheel_pidController.setFF(0); // Using motor model
    flywheel_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Stop by default
    final ShooterFlyWheel subsystem = this;
    this.setDefaultCommand(new Command() {
      @Override
      public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> set = new HashSet<Subsystem>();
        set.add(subsystem);
        return set;
      }

      @Override
      public void execute() {
        // System.out.println("executing default command");
        subsystem.stop();
      }
    });
    flywheelMaster.burnFlash();
    flywheelFollower.burnFlash();
  }

  @Override
  public void periodic() {
    if (flywheelMaster == null) {
      return;
    }
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = P.get();
    double i = I.get();
    double d = D.get();
    double max = maxOutput.get();
    double min = minOutput.get();

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      flywheel_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      flywheel_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      flywheel_pidController.setD(d);
      kD = d;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      flywheel_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    double currentOpenLoopRampRate = SmartDashboard.getNumber("Shooter FlyWheel/Open Loop Ramp Rate",
        defaultOpenLoopRampRate);
    if (lastOpenLoopRampRate != null && currentOpenLoopRampRate != lastOpenLoopRampRate) {
      flywheelMaster.setOpenLoopRampRate(currentOpenLoopRampRate);
      lastOpenLoopRampRate = currentOpenLoopRampRate;
    }
    if (Constants.tuningMode) {
      SmartDashboard.putNumber("Shooter FlyWheel/speed", getSpeed());
      SmartDashboard.putNumber("Shooter FlyWheel/applied output", flywheelMaster.getAppliedOutput());
    }
    SmartDashboard.putBoolean("Shooter FlyWheel/At Speed", atSetpoint());

    setFlyWheelSpeed.set(getSpeed());

    // Update shooter LED
    double targetRpm = closedLoopVelocityProfiler.getSetpointGoal();
    double currentRpm = getSpeed();
    OILEDState shooterLEDState = OILEDState.OFF;
    if (currentRpm > targetRpm * atSetpointThreshold) {
      shooterLEDState = OILEDState.MED;
    } else if (currentRpm > targetRpm * LEDSlowPulseThreshold) {
      shooterLEDState = OILEDState.PULSE_FAST;
    } else if (currentRpm > 0) {
      shooterLEDState = OILEDState.BLINK_SLOW;
    }
    if (shooterLEDState != lastShooterLEDState) {
      updateLED.update(OILED.SHOOTER_SHOOT, shooterLEDState);
      lastShooterLEDState = shooterLEDState;
    }

    // Closed loop control logic
    if (!openLoopControl) {
      // Update setpoint
      double rpmSetpoint = closedLoopVelocityProfiler.getSetpoint();
      double ffVolts = feedForwardModel.calculate(rpmSetpoint);
      setpoint = rpmSetpoint / MULTIPLIER;
      flywheel_pidController.setReference(setpoint, ControlType.kVelocity, 0, ffVolts);
      if (Constants.tuningMode) {
        SmartDashboard.putNumber("Shooter FlyWheel/target setpoint", targetRpm);
        SmartDashboard.putNumber("Shooter FlyWheel/current setpoint", rpmSetpoint);
      }

      if (getSpeed() < closedLoopVelocityProfiler.getSetpointGoal() * accurateMinTheshold
          || getSpeed() > closedLoopVelocityProfiler.getSetpointGoal() * accurateMaxTheshold) {
        accurateInTimer.reset();
      }

      if (accurateInTimer.hasElapsed(accurateInTime)) {
        accurateGraceTimer.reset();
      }
      accurateReady = !accurateGraceTimer.hasElapsed(accurateFeedGrace);
    } else {
      accurateReady = false;
    }
  }

  public void stop() {
    if (flywheelMaster == null) {
      return;
    }
    openLoopControl = true;
    flywheelMaster.stopMotor();
    flywheelFollower.stopMotor();
    closedLoopVelocityProfiler.reset();
    updateRunningLEDs(false);
  }

  public void setShooterRPM(double rpm) {
    if (flywheelMaster == null) {
      return;
    }
    if (openLoopControl) {
      closedLoopVelocityProfiler.setSetpointGoal(rpm, getSpeed());
    } else {
      closedLoopVelocityProfiler.setSetpointGoal(rpm);
    }
    openLoopControl = false;
    updateRunningLEDs(rpm != 0);
  }

  public void run(double power) {
    if (flywheelMaster == null) {
      return;
    }
    openLoopControl = true;
    flywheelMaster.set(power);
    updateRunningLEDs(power != 0);
  }

  public double getSpeed() {
    if (flywheelMaster == null) {
      return 0;
    }
    return flywheelEncoder.getVelocity() * MULTIPLIER;
  }

  public boolean atSetpoint() {
    if (openLoopControl) {
      return false;
    } else {
      return getSpeed() > closedLoopVelocityProfiler.getSetpointGoal() * atSetpointThreshold;
    }
  }

  public boolean readyForAccurateFeed() {
    return accurateReady;
  }

  public boolean safeToFeed() {
    return getSpeed() > safeFeedThreshold;
  }

  private void updateRunningLEDs(boolean running) {
    updateLED.update(OILED.SHOOTER_RUN, running ? OILEDState.MED : OILEDState.OFF);
    updateLED.update(OILED.SHOOTER_STOP, running ? OILEDState.OFF : OILEDState.MED);
  }
}
