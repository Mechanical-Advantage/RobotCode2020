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
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.PressureSensor;
import frckit.util.GeomUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PointAtTargetAndShootTrenchRun extends ParallelDeadlineGroup {

  private static final double powerCellLineY = Constants.fieldWidth / -2 + Constants.trenchRunWidth / 2;
  private static final Pose2d trenchStart = new Pose2d(Constants.fieldLength / 2 + Constants.trenchRunLength / 2,
      powerCellLineY, Rotation2d.fromDegrees(180));
  private static final Pose2d trenchEnd = new Pose2d(Constants.fieldLength / 2, powerCellLineY,
      Rotation2d.fromDegrees(180));
  private static final Pose2d secondShotPosition = new Pose2d(trenchStart.getX() - 24, powerCellLineY,
      Rotation2d.fromDegrees(180));

  private static final Translation2d trenchVelocityConstraintBottomLeft = new Translation2d(
      Constants.fieldLength / 2 - Constants.trenchRunLength / 2, Constants.fieldWidth / -2);
  private static final Translation2d trenchVelocityConstraintTopRight = new Translation2d(
      Constants.fieldLength / 2 + Constants.trenchRunLength / 2, Constants.fieldWidth / -2 + Constants.trenchRunWidth);
  private static final RectangularRegionConstraint trenchVelocityConstraint = new RectangularRegionConstraint(
      GeomUtil.inchesToMeters(trenchVelocityConstraintBottomLeft),
      GeomUtil.inchesToMeters(trenchVelocityConstraintTopRight), new MaxVelocityConstraint(Units.inchesToMeters(80)));

  /**
   * Creates a new PointAtTargetAndShootTrenchRun.
   */
  public PointAtTargetAndShootTrenchRun(DriveTrainBase driveTrain, RobotOdometry odometry, LimelightInterface limelight,
      AHRS ahrs, Hopper hopper, ShooterRoller roller, ShooterFlyWheel flywheel, ShooterHood hood, Intake intake,
      PressureSensor pressureSensor, UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new ParallelCommandGroup(new PointAtTarget(driveTrain, limelight, ahrs),
            new WaitCommand(1).andThen(new WaitCommand(6).withInterrupt(() -> flywheel.atSetpoint())))
                .andThen(
                    new ParallelRaceGroup(new RunHopper(hopper), new RunShooterRoller(roller),
                        new StartEndCommand(() -> limelight.setLEDMode(LimelightLEDMode.ON),
                            () -> limelight.setLEDMode(LimelightLEDMode.OFF), limelight),
                        new WaitCommand(1.5)),
                    new TurnToAngle(driveTrain, ahrs, 135, true, 15), new InstantCommand(intake::extend),
                    new NewRunMotionProfile(driveTrain, odometry, List.of(trenchStart, trenchEnd), 0, false, false,
                        List.of(trenchVelocityConstraint)).deadlineWith(new RunIntakeForwards(intake)),
                    new InstantCommand(intake::retract),
                    new NewRunMotionProfile(driveTrain, odometry, 0, List.of(trenchEnd, secondShotPosition), 0, true,
                        false),
                    new TurnToAngle(driveTrain, ahrs, -15, true, 5), new InstantCommand(intake::extend),
                    new PointAtTarget(driveTrain, limelight, ahrs),
                    new ParallelRaceGroup(new RunHopper(hopper), new RunShooterRoller(roller), new WaitCommand(5))),
        new RunShooterAtDistance(flywheel, hood, odometry, true));
  }
}
