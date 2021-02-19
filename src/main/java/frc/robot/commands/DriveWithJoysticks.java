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

  private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or
  // of current gear
  private static final double rightStickScale = 0.5; // Factor of right stick when added
  private static final double curvatureTurnSensitivity = 1.5; // Greater than 1 allows for reverse output on inner side

  private DoubleSupplier oiLeftDriveX;
  private DoubleSupplier oiLeftDriveY;
  private DoubleSupplier oiLeftDriveTrigger;
  private DoubleSupplier oiRightDriveX;
  private DoubleSupplier oiRightDriveY;
  private DoubleSupplier oiRightDriveTrigger;
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
      DoubleSupplier getDeadband, BooleanSupplier sniperMode, DoubleSupplier sniperLevel,
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outputLeft = 0, outputRight = 0;
    double baseDrive;
    double turnSpeed;
    switch (joystickChooser.getSelected()) {
      case Tank:
        outputRight = processJoystickAxis(oiRightDriveY.getAsDouble(), true);
        outputLeft = processJoystickAxis(oiLeftDriveY.getAsDouble(), true);
        break;
      case SplitArcade:
        baseDrive = processJoystickAxis(oiLeftDriveY.getAsDouble(), true);
        outputRight = baseDrive - processJoystickAxis(oiRightDriveX.getAsDouble(), false);
        outputLeft = baseDrive + processJoystickAxis(oiRightDriveX.getAsDouble(), false);
        break;
      case SplitArcadeSouthpaw:
        baseDrive = processJoystickAxis(oiRightDriveY.getAsDouble(), true);
        outputRight = baseDrive - processJoystickAxis(oiLeftDriveX.getAsDouble(), false);
        outputLeft = baseDrive + processJoystickAxis(oiLeftDriveX.getAsDouble(), false);
        break;
      case Curvature:
        baseDrive = processJoystickAxis(oiLeftDriveY.getAsDouble(), true);
        turnSpeed = processJoystickAxis(oiRightDriveX.getAsDouble(), false);

        if (baseDrive != 0) {
          double maxBaseDrive = 1 / (1 + (turnSpeed * curvatureTurnSensitivity)); // Max speed where no output >1
          baseDrive = MathUtil.clamp(baseDrive, maxBaseDrive * -1, maxBaseDrive);
          turnSpeed = Math.abs(baseDrive) * turnSpeed * curvatureTurnSensitivity;
        }
        outputRight = baseDrive - turnSpeed;
        outputLeft = baseDrive + turnSpeed;
        break;
      case CurvatureSouthpaw:
        baseDrive = processJoystickAxis(oiRightDriveY.getAsDouble(), true);
        turnSpeed = processJoystickAxis(oiLeftDriveX.getAsDouble(), false);

        if (baseDrive != 0) {
          double maxBaseDrive = 1 / (1 + (turnSpeed * curvatureTurnSensitivity)); // Max speed where no output >1
          baseDrive = MathUtil.clamp(baseDrive, maxBaseDrive * -1, maxBaseDrive);
          turnSpeed = Math.abs(baseDrive) * turnSpeed * curvatureTurnSensitivity;
        }
        outputRight = baseDrive - turnSpeed;
        outputLeft = baseDrive + turnSpeed;
        break;
      case Trigger:
        baseDrive = oiRightDriveTrigger.getAsDouble() - oiLeftDriveTrigger.getAsDouble();
        baseDrive = baseDrive * Math.abs(baseDrive);
        turnSpeed = processJoystickAxis(oiLeftDriveX.getAsDouble(), false)
            + (processJoystickAxis(oiRightDriveX.getAsDouble(), false) * rightStickScale);

        outputRight = baseDrive - turnSpeed;
        outputLeft = baseDrive + turnSpeed;
        break;
      case TriggerCurvature:
        baseDrive = oiRightDriveTrigger.getAsDouble() - oiLeftDriveTrigger.getAsDouble();
        baseDrive = baseDrive * Math.abs(baseDrive);
        turnSpeed = processJoystickAxis(oiLeftDriveX.getAsDouble(), false)
            + (processJoystickAxis(oiRightDriveX.getAsDouble(), false) * rightStickScale);

        if (baseDrive != 0) {
          double maxBaseDrive = 1 / (1 + (turnSpeed * curvatureTurnSensitivity)); // Max speed where no output >1
          baseDrive = MathUtil.clamp(baseDrive, maxBaseDrive * -1, maxBaseDrive);
          turnSpeed = Math.abs(baseDrive) * turnSpeed * curvatureTurnSensitivity;
        }
        outputRight = baseDrive - turnSpeed;
        outputLeft = baseDrive + turnSpeed;
        break;
    }

    if (oiSniperMode.getAsBoolean()) {
      double multiplierLevel = getMultiplierForSniperMode();
      outputLeft *= multiplierLevel;
      outputRight *= multiplierLevel;
    }

    if (joysticksReversed) {
      outputLeft = outputRight * -1;
      outputRight = outputLeft * -1;
    }

    driveSubsystem.drive(MathUtil.clamp(outputLeft, -1, 1), MathUtil.clamp(outputRight, -1, 1), alwaysUseHighMaxVel);
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
    Tank, SplitArcade, SplitArcadeSouthpaw, Curvature, CurvatureSouthpaw, Trigger, TriggerCurvature;
  }
}
