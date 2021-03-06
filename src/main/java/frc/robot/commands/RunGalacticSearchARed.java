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
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotOdometry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunGalacticSearchARed extends SequentialCommandGroup {

  NewRunMotionProfile mp;
  private static final double markerDiameterZones = 4;
  private static final double markerDiameterBalls = 7;
  private static final Color markerColorZones = Color.BLACK;
  private static final Color markerColorBalls = Color.RED;

  /** Creates a new RunGalacticSearchARed. */
  public RunGalacticSearchARed(RobotOdometry odometry, DriveTrainBase driveTrain, Intake intake) {
    // new Pose2d(30, 90, Rotation2d.fromDegrees(10)) <- center start
    mp = new NewRunMotionProfile(driveTrain, odometry, new Pose2d(48, 120, Rotation2d.fromDegrees(-35)), 0,
        List.of(new Translation2d(90, 90), new Translation2d(150, 70), new Translation2d(190, 150)),
        new Pose2d(320, 150, new Rotation2d()), Double.MAX_VALUE, false, false);
    // Add your addCommands(new FooCommand(), new BarCommand());
    if (intake != null) {
      addCommands(new InstantCommand(() -> odometry.setPosition(new Pose2d(48, 120, Rotation2d.fromDegrees(-35)))),
          new InstantCommand(() -> intake.extend()), mp.deadlineWith(new RunIntakeForwards(intake)),
          new InstantCommand(() -> driveTrain.stop()));
    }
  }

  public static void main(String[] args) {
    Constants.setRobot(RobotType.ROBOT_2020);
    RunGalacticSearchARed cmd = new RunGalacticSearchARed(null, null, null);
    cmd.mp.visualize(80.0,
        List.of(new TrajectoryMarker(new Translation2d(30, 60), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(30, 120), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(330, 60), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(330, 120), markerDiameterZones, markerColorZones),
            new TrajectoryMarker(new Translation2d(90, 90), markerDiameterBalls, markerColorBalls),
            new TrajectoryMarker(new Translation2d(150, 60), markerDiameterBalls, markerColorBalls),
            new TrajectoryMarker(new Translation2d(180, 150), markerDiameterBalls, markerColorBalls)));
  }
}
