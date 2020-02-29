/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.PressureSensor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PointAtTargetAndShootTrenchRun extends ParallelDeadlineGroup {

  private static final double powerCellLineX = Constants.fieldWidth / 2 - Constants.trenchRunWidth / 2;
  private static final Pose2d endPose = new Pose2d(powerCellLineX, Constants.fieldLength / 2,
      Rotation2d.fromDegrees(180));
  private static final Translation2d trenchStart = new Translation2d(powerCellLineX,
      Constants.fieldLength / 2 + Constants.trenchRunLength / 2);

  /**
   * Creates a new PointAtTargetAndShootTrenchRun.
   */
  public PointAtTargetAndShootTrenchRun(DriveTrainBase driveTrain, RobotOdometry odometry, LimelightInterface limelight,
      AHRS ahrs, Hopper hopper, ShooterRoller roller, ShooterFlyWheel flywheel, ShooterHood hood,
      PressureSensor pressureSensor, UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new PointAtTarget(driveTrain, limelight, ahrs)
        .alongWith(new SetShooterHoodMiddleTop(hood, pressureSensor, false, updateLED, setHoodLCD))
        .alongWith(new WaitCommand(7).withInterrupt(() -> flywheel.getSpeed() > 6000))
        .andThen(new RunHopper(hopper).alongWith(new RunShooterRoller(roller)).withTimeout(2))
        .andThen(new RunMotionProfile(driveTrain, odometry, List.of(trenchStart), endPose, 0, false, false))
        .andThen(new TurnToAngle(driveTrain, ahrs, -20, true)).andThen(new PointAtTarget(driveTrain, limelight, ahrs))
        .andThen(new RunHopper(hopper).alongWith(new RunShooterRoller(roller)).withTimeout(5)),
        new RunShooterFlyWheel(flywheel));
  }
}
