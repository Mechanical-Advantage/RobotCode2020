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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.tools.pathview.TrajectoryMarker;
import frc.robot.util.trajectory.CirclePath;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoNavBounce extends SequentialCommandGroup {

  NewRunMotionProfile mp1;
  NewRunMotionProfile mp2;
  NewRunMotionProfile mp3;
  NewRunMotionProfile mp4;
  private static final double markerDiameter = 4;
  private static final double markerDiameterCrates = 12;
  private static final Color markerColorMain = Color.BLACK;
  private static final Color markerColorCrates = new Color(240, 190, 0);
  private static final Color markerColorStart = new Color(0, 200, 0);;
  private static final Color markerColorEnd = Color.RED;

  /** Creates a new RunAutoNavBounce. */
  public RunAutoNavBounce(RobotOdometry odometry, DriveTrainBase driveTrain, Intake intake) {
    mp1 = new NewRunMotionProfile(driveTrain, odometry, 0,
        List.of(new Pose2d(30, 90, new Rotation2d()), new Pose2d(90, 118, Rotation2d.fromDegrees(100))), 0, false,
        false, new ArrayList<>());
    mp2 = new NewRunMotionProfile(driveTrain, odometry, 0,
        List.of(new Pose2d(90, 118, Rotation2d.fromDegrees(100)),
            new CirclePath(new Translation2d(150, 60), 30, Rotation2d.fromDegrees(-160), new Rotation2d(), false),
            new Pose2d(180, 130, Rotation2d.fromDegrees(-90))),
        0, true, false, new ArrayList<>());
    mp3 = new NewRunMotionProfile(driveTrain, odometry, 0,
        List.of(new Pose2d(180, 130, Rotation2d.fromDegrees(-90)), new Pose2d(180, 60, Rotation2d.fromDegrees(-90)),
            new Pose2d(270, 60, Rotation2d.fromDegrees(90)), new Pose2d(270, 118, Rotation2d.fromDegrees(90))),
        0, false, false, new ArrayList<>());
    mp4 = new NewRunMotionProfile(driveTrain, odometry, 0,
        List.of(new Pose2d(270, 118, Rotation2d.fromDegrees(90)), new Pose2d(285, 90, Rotation2d.fromDegrees(145))),
        Double.MAX_VALUE, true, false, new ArrayList<>());
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))),
        new InstantCommand(() -> intake.extend()), mp1, mp2, mp3, mp4, new InstantCommand(() -> driveTrain.stop()));
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunAutoNavBounce cmd = new RunAutoNavBounce(null, null, null);
    NewRunMotionProfile
        .runVisualizer(
            List.of(cmd.mp1.visualizerGetTrajectory(), cmd.mp2.visualizerGetTrajectory(),
                cmd.mp3.visualizerGetTrajectory(), cmd.mp4.visualizerGetTrajectory()),
            cmd.mp1.visualizerGetTrackWidth(), 80,
            List.of(new TrajectoryMarker(new Translation2d(90, 150), markerDiameterCrates, markerColorCrates),
                new TrajectoryMarker(new Translation2d(180, 150), markerDiameterCrates, markerColorCrates),
                new TrajectoryMarker(new Translation2d(270, 150), markerDiameterCrates, markerColorCrates),
                new TrajectoryMarker(new Translation2d(30, 120), markerDiameter, markerColorStart),
                new TrajectoryMarker(new Translation2d(60, 120), markerDiameter, markerColorStart),
                new TrajectoryMarker(new Translation2d(120, 120), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(150, 120), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(210, 120), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(240, 120), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(300, 120), markerDiameter, markerColorEnd),
                new TrajectoryMarker(new Translation2d(330, 120), markerDiameter, markerColorEnd),
                new TrajectoryMarker(new Translation2d(30, 60), markerDiameter, markerColorStart),
                new TrajectoryMarker(new Translation2d(60, 60), markerDiameter, markerColorStart),
                new TrajectoryMarker(new Translation2d(90, 60), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(150, 60), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(210, 60), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(240, 60), markerDiameter, markerColorMain),
                new TrajectoryMarker(new Translation2d(300, 60), markerDiameter, markerColorEnd),
                new TrajectoryMarker(new Translation2d(330, 60), markerDiameter, markerColorEnd),
                new TrajectoryMarker(new Translation2d(90, 30), markerDiameter, markerColorMain)));
  }
}
