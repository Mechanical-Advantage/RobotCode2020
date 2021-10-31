// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.subsystems.drive.DriveTrainBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PowerPortChallengeAuto extends SequentialCommandGroup {

  private static final double stopDistance = Constants.fieldLength - 215;
  private static final double shootDistance = Constants.fieldLength - 215;
  private static final Pose2d loadPosition = new Pose2d(Constants.fieldLength - 300,
      Constants.visionTargetHorizDist * -1 + 35, Rotation2d.fromDegrees(-10));

  /** Creates a new PowerPortChallengeAuto. */
  public PowerPortChallengeAuto(DriveTrainBase driveTrain, RobotOdometry odometry, Intake intake, Hopper hopper,
      ShooterRoller roller, ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, LimelightInterface limelight) {
    DriveToTarget driveToTarget = new DriveToTarget(driveTrain, odometry, stopDistance);
    WaitUntilCommand readyToShoot = new WaitUntilCommand(() -> shooterFlyWheel.atSetpoint()
        && driveToTarget.angularReady() && odometry.getCurrentPose().getX() > shootDistance);
    addCommands(
        new ParallelDeadlineGroup(
            readyToShoot.andThen(new RunHopper(hopper).alongWith(new RunShooterRoller(roller)).withTimeout(1.5)),
            driveToTarget,
            new InstantCommand(() -> shooterHood.setTargetPosition(HoodPosition.BACK_LINE))
                .andThen(new RunShooterAtDistance(shooterFlyWheel, shooterHood, odometry, false, () -> false)),
            new StartEndCommand(intake::retract, intake::extend, intake),
            new StartEndCommand(() -> limelight.setLEDMode(LimelightLEDMode.ON),
                () -> limelight.setLEDMode(LimelightLEDMode.OFF), limelight)),
        new NewRunMotionProfile(driveTrain, odometry, List.of(loadPosition), 0, true, false, new ArrayList<>()));
  }
}