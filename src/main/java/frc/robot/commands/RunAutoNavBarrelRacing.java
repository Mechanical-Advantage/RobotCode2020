// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.awt.Color;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.tools.pathview.TrajectoryMarker;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoNavBarrelRacing extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  private static final double markerDiameter = 4;
  private static final Color markerColorMain = Color.BLACK;
  private static final Color markerColorStart = new Color(0, 200, 0);

  /** Creates a new RunAutoNavBarrelRacing. */
  public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveTrainBase driveTrain) {
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0,
        List.of(new Pose2d(30.0, 90.0, new Rotation2d()),
            new CirclePath(new Translation2d(150, 60), 30, new Rotation2d(), Rotation2d.fromDegrees(-180), true),
            new CirclePath(new Translation2d(240, 120), 30, new Rotation2d(), Rotation2d.fromDegrees(180), false),
            new CirclePath(new Translation2d(300, 60), 30, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90),
                false),
            new Pose2d(150.0, 90, Rotation2d.fromDegrees(180)), new Pose2d(42.0, 90.0, Rotation2d.fromDegrees(180))),
        Double.MAX_VALUE, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))), mp,
        new InstantCommand(() -> driveTrain.stop()));
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunAutoNavBarrelRacing cmd = new RunAutoNavBarrelRacing(null, null);
    NewRunMotionProfile.runVisualizer(List.of(cmd.mp.visualizerGetTrajectory()), cmd.mp.visualizerGetTrackWidth(), 80,
        List.of(new TrajectoryMarker(new Translation2d(30, 120), markerDiameter, markerColorStart),
            new TrajectoryMarker(new Translation2d(60, 120), markerDiameter, markerColorStart),
            new TrajectoryMarker(new Translation2d(30, 60), markerDiameter, markerColorStart),
            new TrajectoryMarker(new Translation2d(60, 60), markerDiameter, markerColorStart),
            new TrajectoryMarker(new Translation2d(150, 60), markerDiameter, markerColorMain),
            new TrajectoryMarker(new Translation2d(240, 120), markerDiameter, markerColorMain),
            new TrajectoryMarker(new Translation2d(300, 60), markerDiameter, markerColorMain)));
  }
}
