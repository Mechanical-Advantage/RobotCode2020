// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

import java.awt.Color;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.RequireCommand;
import frckit.tools.pathview.TrajectoryMarker;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchBBlue extends ParallelRaceGroup {

  NewRunMotionProfile mp;
  private static final double markerDiameterZones = 4;
  private static final double markerDiameterBalls = 7;
  private static final Color markerColorZones = Color.BLACK;
  private static final Color markerColorBalls = Color.BLUE;
  private static final TrajectoryConstraint pickupConstraint = new MaxVelocityConstraint(80);

  /** Creates a new RunGalacticSearchBBlue. */
  public RunGalacticSearchBBlue(RobotOdometry odometry, DriveTrainBase driveTrain, Intake intake) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(-20)) <- center start
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(48, 30, new Rotation2d()), 0,
        List.of(new Translation2d(180, 70), new Translation2d(240, 110)),
        new Pose2d(320, 55, Rotation2d.fromDegrees(-35)), 100, false, false,
        List.of(new EllipticalRegionConstraint(new Translation2d(155, 65), 20, 20, new Rotation2d(), pickupConstraint),
            new EllipticalRegionConstraint(new Translation2d(290, 70), 20, 20, new Rotation2d(), pickupConstraint)));
    // Add your addCommands(new FooCommand(), new BarCommand());
    if (intake != null) {
      addCommands(new SequentialCommandGroup(
          new InstantCommand(() -> odometry.setPosition(new Pose2d(48, 30, new Rotation2d()))),
          new InstantCommand(() -> intake.extend()), mp.deadlineWith(new RunIntakeForwards(intake)),
          new InstantCommand(() -> driveTrain.stop())), new RequireCommand(odometry));
    }
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunGalacticSearchBBlue cmd = new RunGalacticSearchBBlue(null, null, null);
    NewRunMotionProfile.runVisualizer(List.of(cmd.mp.visualizerGetTrajectory()), cmd.mp.visualizerGetTrackWidth(), 80.0,
        List.of(new TrajectoryMarker(new Translation2d(30, 60), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(30, 120), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(330, 60), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(330, 120), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(180, 60), markerDiameterBalls, markerColorBalls),
            new TrajectoryMarker(new Translation2d(240, 120), markerDiameterBalls, markerColorBalls),
            new TrajectoryMarker(new Translation2d(300, 60), markerDiameterBalls, markerColorBalls)));
  }
}
