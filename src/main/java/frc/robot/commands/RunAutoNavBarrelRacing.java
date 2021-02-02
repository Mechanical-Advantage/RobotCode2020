// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

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
public class RunAutoNavBarrelRacing extends SequentialCommandGroup {

  NewRunMotionProfile mp;

  /** Creates a new RunAutoNavBarrelRacing. */
  public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveTrainBase driveTrain) {
    mp = new NewRunMotionProfile(driveTrain, odometry, 0.0, List.of(new Pose2d(30.0, 90.0, new Rotation2d()),
        new CirclePath(new Translation2d(150, 60), 30, Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), true),
        new CirclePath(new Translation2d(240, 120), 30, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-135),
            false),
        new CirclePath(new Translation2d(300, 60), 30, Rotation2d.fromDegrees(-135), Rotation2d.fromDegrees(90), false),
        new Pose2d(240.0, 89.95, Rotation2d.fromDegrees(180)), new Pose2d(150.0, 90.05, Rotation2d.fromDegrees(180)),
        new Pose2d(30.0, 90.0, Rotation2d.fromDegrees(180))), 0.0, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))), mp);
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunAutoNavBarrelRacing cmd = new RunAutoNavBarrelRacing(null, null);
    cmd.mp.visualize(2.0,
        List.of(new Translation2d(30, 120), new Translation2d(60, 120), new Translation2d(30, 60),
            new Translation2d(60, 60), new Translation2d(150, 60), new Translation2d(240, 120),
            new Translation2d(300, 60)));
  }
}
