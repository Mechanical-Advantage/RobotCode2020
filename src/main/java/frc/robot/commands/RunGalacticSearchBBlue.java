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
public class RunGalacticSearchBBlue extends SequentialCommandGroup {

  /** Creates a new RunGalacticSearchABlue. */
  public RunGalacticSearchBBlue(RobotOdometry odometry, DriveTrainBase driveTrain) {
    // Add your addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(30, 65, new Rotation2d()))),
        new RunMotionProfile(driveTrain, odometry, new Pose2d(30, 65, new Rotation2d()), 0,
            List.of(new Translation2d(170, 65), new Translation2d(240, 115)),
            new Pose2d(330, 30, Rotation2d.fromDegrees(-45)), 0, false, false, false));
  }
}