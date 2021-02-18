// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.commands.NewRunMotionProfile.CirclePath;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunHyperdriveLightspeedCircuit extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /**
   * Creates a new RunHyperdriveLightspeedCircuit. This path is NOT part of the
   * stndard autonomous at home challenges.
   */
  public RunHyperdriveLightspeedCircuit(RobotOdometry odometry, DriveTrainBase driveTrain) {
    List<Object> lap = List.of(new Pose2d(105, 150, new Rotation2d()), new Pose2d(195, 90, new Rotation2d()),
        new CirclePath(new Translation2d(270, 120), 30, Rotation2d.fromDegrees(160), Rotation2d.fromDegrees(10), true),
        new CirclePath(new Translation2d(300, 60), 30, Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(-90), true),
        new Pose2d(195, 30, Rotation2d.fromDegrees(180)), new Pose2d(105, 90, Rotation2d.fromDegrees(-180)));
    List<Object> waypoints = new ArrayList<>();
    waypoints.add(new Pose2d(60, 90, Rotation2d.fromDegrees(90)));
    waypoints.addAll(lap);
    waypoints.add(
        new CirclePath(new Translation2d(90, 120), 30, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90), true));
    waypoints.addAll(lap);
    waypoints.add(new Pose2d(55, 60, Rotation2d.fromDegrees(-135)));
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0, waypoints, Double.MAX_VALUE, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(60, 90, Rotation2d.fromDegrees(90)))), mp,
        new InstantCommand(() -> driveTrain.stop()));
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunHyperdriveLightspeedCircuit cmd = new RunHyperdriveLightspeedCircuit(null, null);
    cmd.mp.visualize(80, List.of(new Translation2d(30, 60), new Translation2d(90, 60), new Translation2d(120, 60),
        new Translation2d(180, 60), new Translation2d(210, 60), new Translation2d(240, 60), new Translation2d(270, 60),
        new Translation2d(300, 60), new Translation2d(270, 90), new Translation2d(30, 120), new Translation2d(90, 120),
        new Translation2d(120, 120), new Translation2d(180, 120), new Translation2d(210, 120),
        new Translation2d(270, 120), new Translation2d(330, 120), new Translation2d(180, 150)));
  }
}
