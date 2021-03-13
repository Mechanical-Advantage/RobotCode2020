// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.LimelightInterface.LimelightLEDMode;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.util.GeomUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PowerPortChallengeAuto extends ParallelDeadlineGroup {

  private static final Pose2d shootPosition = new Pose2d(Constants.fieldLength - 153,
      Constants.visionTargetHorizDist * -1, new Rotation2d());

  /** Creates a new PowerPortChallengeAuto. */
  public PowerPortChallengeAuto(DriveTrainBase driveTrain, RobotOdometry odometry, Intake intake, Hopper hopper,
      ShooterRoller roller, ShooterFlyWheel shooterFlyWheel, ShooterHood shooterHood, LimelightInterface limelight) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new SequentialCommandGroup(new DriveToPoint(driveTrain, odometry, shootPosition),
        new RunHopper(hopper).alongWith(new RunShooterRoller(roller)).withTimeout(1.5)));
    addCommands(new RunShooterAtDistance(shooterFlyWheel, shooterHood, shootPosition.getTranslation(), true),
        new StartEndCommand(intake::retract, intake::extend, intake),
        new StartEndCommand(() -> limelight.setLEDMode(LimelightLEDMode.ON),
            () -> limelight.setLEDMode(LimelightLEDMode.OFF), limelight));
  }
}
