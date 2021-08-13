// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.tools.pathview.TrajectoryMarker;
import frc.robot.util.RequireCommand;
import frc.robot.util.trajectory.CirclePath;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoNavSlalom extends ParallelRaceGroup {

  NewRunMotionProfile mp1;
  NewRunMotionProfile mp2;
  NewRunMotionProfile mp3;
  NewRunMotionProfile mp4;
  private static final Double velocityOverride = 150.0;
  private static final Double accelerationOverride = 250.0;
  private static final Double centripetalAccelerationOverride = Double.MAX_VALUE;
  private static final double markerDiameter = 4;
  private static final Color markerColor = Color.BLACK;

  /** Creates a new RunAutoNavSlalom. */
  public RunAutoNavSlalom(RobotOdometry odometry, DriveTrainBase driveTrain) {
    mp1 = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(36, 24, Rotation2d.fromDegrees(28)), new Pose2d(78.0, 60.0, Rotation2d.fromDegrees(70.0)),
            new Pose2d(160, 70, new Rotation2d()), new Pose2d(210, 70, new Rotation2d())),
        velocityOverride, false, false, new ArrayList<>(), velocityOverride, accelerationOverride,
        centripetalAccelerationOverride, false);
    mp2 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
        List.of(new Pose2d(195, 90, new Rotation2d()), new Pose2d(230, 90, new Rotation2d()),
            new CirclePath(new Translation2d(300, 76), 33, Rotation2d.fromDegrees(-160), Rotation2d.fromDegrees(160),
                false)),
        velocityOverride, false, false, new ArrayList<>(), velocityOverride, accelerationOverride,
        centripetalAccelerationOverride, false);
    mp3 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
        List.of(new Pose2d(305, 85, Rotation2d.fromDegrees(180)), new Pose2d(270, 60, Rotation2d.fromDegrees(-135)),
            new Pose2d(210, 50, Rotation2d.fromDegrees(180))),
        velocityOverride, false, false, new ArrayList<>(), velocityOverride, accelerationOverride,
        centripetalAccelerationOverride, false);
    mp4 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
        List.of(new Pose2d(250, 35, Rotation2d.fromDegrees(-170)), new Pose2d(150, 30, Rotation2d.fromDegrees(180)),
            new Pose2d(110, 60, Rotation2d.fromDegrees(135)), new Pose2d(40, 52, Rotation2d.fromDegrees(180))),
        velocityOverride, false, false, new ArrayList<>(), velocityOverride, accelerationOverride,
        centripetalAccelerationOverride, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    if (odometry != null && driveTrain != null) {
      addCommands(new SequentialCommandGroup(
          new InstantCommand(() -> odometry.setPosition(new Pose2d(36, 24, Rotation2d.fromDegrees(28)))), mp1,
          new InstantCommand(() -> odometry.setPosition(new Translation2d(195, 90))), mp2,
          new InstantCommand(() -> odometry.setPosition(new Translation2d(305, 85))), mp3,
          new InstantCommand(() -> odometry.setPosition(new Translation2d(250, 33))), mp4,
          new InstantCommand(() -> driveTrain.stop())), new RequireCommand(odometry));
    }
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunAutoNavSlalom cmd = new RunAutoNavSlalom(null, null);
    NewRunMotionProfile
        .runVisualizer(
            List.of(cmd.mp1.visualizerGetTrajectory(), cmd.mp2.visualizerGetTrajectory(),
                cmd.mp3.visualizerGetTrajectory(), cmd.mp4.visualizerGetTrajectory()),
            cmd.mp1.visualizerGetTrackWidth(), 80,
            List.of(new TrajectoryMarker(new Translation2d(30, 120), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(60, 120), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(30, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(60, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(120, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(150, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(180, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(210, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(240, 60), markerDiameter, markerColor),
                new TrajectoryMarker(new Translation2d(300, 60), markerDiameter, markerColor)));
  }
}
