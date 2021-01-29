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
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutoNavBounce extends SequentialCommandGroup {

  /** Creates a new RunGalacticSearchABlue. */
  public RunAutoNavBounce(RobotOdometry odometry, DriveTrainBase driveTrain) {
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 90, new Rotation2d()))),
        new RunMotionProfile(driveTrain, odometry, List.of(new Translation2d(70, 90)),
            new Pose2d(90, 150, Rotation2d.fromDegrees(90)), 0, false, false, false),
        new RunMotionProfile(driveTrain, odometry, new Pose2d(90, 150, new Rotation2d(90)), 0,
            List.of(new Translation2d(120, 60), new Translation2d(150, 30), new Translation2d(180, 60)),
            new Pose2d(180, 150, Rotation2d.fromDegrees(-90)), 0, true, false, false),
        new RunMotionProfile(driveTrain, odometry, new Pose2d(180, 150, new Rotation2d(-90)), 0,
            List.of(new Translation2d(180, 60), new Translation2d(200, 38), new Translation2d(270, 38)),
            new Pose2d(280, 150, Rotation2d.fromDegrees(90)), 0, false, false, false),
        new RunMotionProfile(driveTrain, odometry, new Pose2d(280, 150, new Rotation2d(90)), 0,
            List.of(new Translation2d(290, 100)), new Pose2d(330, 100, Rotation2d.fromDegrees(180)), 0, true, false,
            false));
  }
}
