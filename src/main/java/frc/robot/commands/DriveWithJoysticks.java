/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.drive.DriveTrainBase;

public class DriveWithJoysticks extends CommandBase {
  /**
   * Creates a new DriveWithJoysticks.
   */

  private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or of
                                                           // current gear
  private static final double rightStickScale = 0.5; // Factor of right stick when added for trigger
  private static final double curvatureTurnSensitivity = 1.5; // Greater than 1 allows for reverse output on inner side
  private static final double hybridCurvatureThreshold = 0.15; // Under this base speed, blend to split arcade

  private DoubleSupplier oiLeftDriveX;
  private DoubleSupplier oiLeftDriveY;
  private DoubleSupplier oiLeftDriveTrigger;
  private DoubleSupplier oiRightDriveX;
  private DoubleSupplier oiRightDriveY;
  private DoubleSupplier oiRightDriveTrigger;
  private BooleanSupplier oiGetQuickTurn;
  private DoubleSupplier oiGetDeadband;
  private BooleanSupplier oiSniperMode;
  private DoubleSupplier oiSniperLevel;
  private DoubleSupplier oiSniperHighLevel;
  private DoubleSupplier oiSniperLowLevel;
  private BooleanSupplier oiSniperLow;
  private BooleanSupplier oiSniperHigh;
  private boolean oiHasDualSniperMode;
  private DriveTrainBase driveSubsystem;
  private SendableChooser<JoystickMode> joystickChooser;

  private boolean joysticksReversed = false;

  public DriveWithJoysticks(DoubleSupplier leftDriveX, DoubleSupplier leftDriveY, DoubleSupplier leftDriveTrigger,
      DoubleSupplier rightDriveX, DoubleSupplier rightDriveY, DoubleSupplier rightDriveTrigger,
      BooleanSupplier getQuickTurn, DoubleSupplier getDeadband, BooleanSupplier sniperMode, DoubleSupplier sniperLevel,
      DoubleSupplier sniperHighLevel, DoubleSupplier sniperLowLevel, BooleanSupplier sniperLow,
      BooleanSupplier sniperHigh, boolean hasDualSniperMode, SendableChooser<JoystickMode> joystickModeChooser,
      DriveTrainBase driveSubsystem) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super();
    addRequirements(driveSubsystem);
    oiLeftDriveX = leftDriveX;
    oiLeftDriveY = leftDriveY;
    oiLeftDriveTrigger = leftDriveTrigger;
    oiRightDriveX = rightDriveX;
    oiRightDriveY = rightDriveY;
    oiRightDriveTrigger = rightDriveTrigger;
    oiGetQuickTurn = getQuickTurn;
    oiGetDeadband = getDeadband;
    oiSniperMode = sniperMode;
    oiSniperLevel = sniperLevel;
    oiSniperHighLevel = sniperHighLevel;
    oiSniperLowLevel = sniperLowLevel;
    oiSniperLow = sniperLow;
    oiSniperHigh = sniperHigh;
    oiHasDualSniperMode = hasDualSniperMode;
    joystickChooser = joystickModeChooser;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Driver Control", true);
  }

  private double processJoystickAxis(double joystickAxis, boolean invert) {
    double deadband = oiGetDeadband.getAsDouble();
    if (Math.abs(joystickAxis) > deadband) {
      double adjustedValue = Math.copySign((Math.abs(joystickAxis) - deadband) / (1 - deadband), joystickAxis);
      return adjustedValue * Math.abs(adjustedValue) * (invert ? -1 : 1);
    } else {
      return 0;
    }
  }

  private static class WheelSpeeds {
    public double left;
    public double right;

    public WheelSpeeds(double left, double right) {
      this.left = left;
      this.right = right;
    }

    public double getLeftProcessed(boolean reversed, double multipler) {
      return (reversed ? MathUtil.clamp(right, -1, 1) * -1 : MathUtil.clamp(left, -1, 1)) * multipler;
    }

    public double getRightProcessed(boolean reversed, double multipler) {
      return (reversed ? MathUtil.clamp(left, -1, 1) * -1 : MathUtil.clamp(right, -1, 1)) * multipler;
    }

    public static WheelSpeeds fromArcade(double baseSpeed, double turnSpeed) {
      return new WheelSpeeds(baseSpeed + turnSpeed, baseSpeed - turnSpeed);
    }

    public static WheelSpeeds fromCurvature(double baseSpeed, double turnSpeed) {
      double maxBaseSpeed = 1 / (1 + (Math.abs(turnSpeed) * curvatureTurnSensitivity)); // Max speed where no output >1
      double baseSpeedLimited = MathUtil.clamp(baseSpeed, maxBaseSpeed * -1, maxBaseSpeed);
      turnSpeed = Math.abs(baseSpeedLimited) * turnSpeed * curvatureTurnSensitivity;
      return new WheelSpeeds(baseSpeedLimited + turnSpeed, baseSpeedLimited - turnSpeed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double primaryX, primaryY, secondaryX, secondaryY; // Primary for driving, secondary for turning
    double leftTrigger = oiLeftDriveTrigger.getAsDouble();
    double rightTrigger = oiRightDriveTrigger.getAsDouble();
    boolean quickTurn = oiGetQuickTurn.getAsBoolean();
    switch (joystickChooser.getSelected()) {
      case SplitArcadeSouthpaw:
      case ManualCurvatureSouthpaw:
      case HybridCurvatureSouthpaw:
        primaryX = processJoystickAxis(oiRightDriveX.getAsDouble(), false);
        primaryY = processJoystickAxis(oiRightDriveY.getAsDouble(), true);
        secondaryX = processJoystickAxis(oiLeftDriveX.getAsDouble(), false);
        secondaryY = processJoystickAxis(oiLeftDriveY.getAsDouble(), true);
        break;
      default:
        primaryX = processJoystickAxis(oiLeftDriveX.getAsDouble(), false);
        primaryY = processJoystickAxis(oiLeftDriveY.getAsDouble(), true);
        secondaryX = processJoystickAxis(oiRightDriveX.getAsDouble(), false);
        secondaryY = processJoystickAxis(oiRightDriveY.getAsDouble(), true);
        break;
    }

    WheelSpeeds outputSpeeds = new WheelSpeeds(0, 0);
    double baseSpeed, turnSpeed, hybridScale;
    WheelSpeeds splitArcadeSpeeds, curvatureSpeeds;
    switch (joystickChooser.getSelected()) {
      case Tank:
        outputSpeeds = new WheelSpeeds(primaryY, secondaryY);
        break;
      case SplitArcade:
      case SplitArcadeSouthpaw:
        outputSpeeds = WheelSpeeds.fromArcade(primaryY, secondaryX);
        break;
      case ManualCurvature:
      case ManualCurvatureSouthpaw:
        if (quickTurn) {
          outputSpeeds = WheelSpeeds.fromArcade(primaryY, secondaryX);
        } else {
          outputSpeeds = WheelSpeeds.fromCurvature(primaryY, secondaryX);
        }
        break;
      case HybridCurvature:
      case HybridCurvatureSouthpaw:
        splitArcadeSpeeds = WheelSpeeds.fromArcade(primaryY, secondaryX);
        curvatureSpeeds = WheelSpeeds.fromCurvature(primaryY, secondaryX);

        hybridScale = Math.abs(primaryY) / hybridCurvatureThreshold;
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        outputSpeeds = new WheelSpeeds(
            (curvatureSpeeds.left * hybridScale) + (splitArcadeSpeeds.left * (1 - hybridScale)),
            (curvatureSpeeds.right * hybridScale) + (splitArcadeSpeeds.right * (1 - hybridScale)));
        break;
      case Trigger:
        baseSpeed = rightTrigger - leftTrigger;
        baseSpeed = baseSpeed * Math.abs(baseSpeed);
        turnSpeed = primaryX + (secondaryX * rightStickScale);

        outputSpeeds = WheelSpeeds.fromArcade(baseSpeed, turnSpeed);
        break;
      case TriggerManualCurvature:
        baseSpeed = rightTrigger - leftTrigger;
        baseSpeed = baseSpeed * Math.abs(baseSpeed);
        turnSpeed = primaryX + (secondaryX * rightStickScale);

        if (quickTurn) {
          outputSpeeds = WheelSpeeds.fromArcade(baseSpeed, turnSpeed);
        } else {
          outputSpeeds = WheelSpeeds.fromCurvature(baseSpeed, turnSpeed);
        }
        break;
      case TriggerHybridCurvature:
        baseSpeed = rightTrigger - leftTrigger;
        baseSpeed = baseSpeed * Math.abs(baseSpeed);
        turnSpeed = primaryX + (secondaryX * rightStickScale);

        splitArcadeSpeeds = WheelSpeeds.fromArcade(baseSpeed, turnSpeed);
        curvatureSpeeds = WheelSpeeds.fromCurvature(baseSpeed, turnSpeed);

        hybridScale = Math.abs(baseSpeed) / hybridCurvatureThreshold;
        hybridScale = hybridScale > 1 ? 1 : hybridScale;
        outputSpeeds = new WheelSpeeds(
            (curvatureSpeeds.left * hybridScale) + (splitArcadeSpeeds.left * (1 - hybridScale)),
            (curvatureSpeeds.right * hybridScale) + (splitArcadeSpeeds.right * (1 - hybridScale)));
        break;
    }

    double multiplierLevel = oiSniperMode.getAsBoolean() ? getMultiplierForSniperMode() : 1;
    double outputLeft = outputSpeeds.getLeftProcessed(joysticksReversed, multiplierLevel);
    double outputRight = outputSpeeds.getRightProcessed(joysticksReversed, multiplierLevel);
    driveSubsystem.drive(outputLeft, outputRight, alwaysUseHighMaxVel);
  }

  public double getMultiplierForSniperMode() {
    double sniperLevel;
    if (oiHasDualSniperMode) {
      if (oiSniperHigh.getAsBoolean()) {
        sniperLevel = oiSniperHighLevel.getAsDouble();
      } else {
        sniperLevel = oiSniperLowLevel.getAsDouble();
      }
    } else {
      sniperLevel = oiSniperLevel.getAsDouble();
    }
    return sniperLevel;
  }

  public void setReversed(boolean reverse) {
    joysticksReversed = reverse;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Driver Control", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static enum JoystickMode {
    Tank, SplitArcade, SplitArcadeSouthpaw, ManualCurvature, ManualCurvatureSouthpaw, HybridCurvature,
    HybridCurvatureSouthpaw, Trigger, TriggerManualCurvature, TriggerHybridCurvature;
  }
}
