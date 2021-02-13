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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
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
  private static final Pose2d trenchStartPose = new Pose2d(trenchStart, Rotation2d.fromDegrees(180));
  private static final Pose2d secondShotPosition = new Pose2d(powerCellLineX, trenchStart.getY() + 36,
      Rotation2d.fromDegrees(180));
  private static final double trenchStartVelocity = 50;

  /**
   * Creates a new PointAtTargetAndShootTrenchRun.
   */
  public PointAtTargetAndShootTrenchRun(DriveTrainBase driveTrain, RobotOdometry odometry, LimelightInterface limelight,
      AHRS ahrs, Hopper hopper, ShooterRoller roller, ShooterFlyWheel flywheel, ShooterHood hood, Intake intake,
      PressureSensor pressureSensor, UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ParallelCommandGroup(new PointAtTarget(driveTrain, limelight, ahrs),
        new SetShooterHoodMiddleTop(hood, pressureSensor, false, updateLED, setHoodLCD),
        new WaitCommand(7).withInterrupt(() -> flywheel.getSpeed() > 5500)).andThen(
            new ParallelRaceGroup(new RunHopper(hopper), new RunShooterRoller(roller), new WaitCommand(1.5)),
            new InstantCommand(intake::extend), new TurnToAngle(driveTrain, ahrs, 135, true, 10),
            new RunMotionProfile(driveTrain, odometry, List.of(), trenchStartPose, trenchStartVelocity, false, false),
            new RunMotionProfile(driveTrain, odometry, List.of(), endPose, 0, false, false)
                .deadlineWith(new RunIntakeForwards(intake)),
            new InstantCommand(intake::retract),
            new RunMotionProfile(driveTrain, odometry, List.of(), secondShotPosition, 0, true, false),
            // .alongWith(new SetShooterHoodMiddleTop(hood, pressureSensor, true, updateLED,
            // setHoodLCD)),
            new TurnToAngle(driveTrain, ahrs, -15, true, 5), new PointAtTarget(driveTrain, limelight, ahrs),
            new ParallelRaceGroup(new RunHopper(hopper), new RunShooterRoller(roller), new WaitCommand(5))),
        new RunShooterFlyWheel(flywheel));
  }
}
