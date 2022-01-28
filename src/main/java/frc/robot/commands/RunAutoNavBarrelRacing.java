// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
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
public class RunAutoNavBarrelRacing extends ParallelRaceGroup {

        NewRunMotionProfile mp1;
        NewRunMotionProfile mp2;
        NewRunMotionProfile mp3;
        NewRunMotionProfile mp4;
        NewRunMotionProfile mp5;
        private static final Double velocityOverride = 150.0;
        private static final Double accelerationOverride = 250.0;
        private static final Double centripetalAccelerationOverride = Double.MAX_VALUE;
        private static final double markerDiameter = 4;
        private static final Color markerColorMain = Color.BLACK;
        private static final Color markerColorStart = new Color(0, 200, 0);

        /** Creates a new RunAutoNavBarrelRacing. */
        public RunAutoNavBarrelRacing(RobotOdometry odometry, DriveTrainBase driveTrain) {
                mp1 = new NewRunMotionProfile(driveTrain, odometry, 0.0,
                                List.of(new Pose2d(41.5, 90.0, new Rotation2d()),
                                                new CirclePath(new Translation2d(132, 60), 30,
                                                                Rotation2d.fromDegrees(90),
                                                                Rotation2d.fromDegrees(90),
                                                                true)),
                                Double.MAX_VALUE, false, false, new ArrayList<>(), velocityOverride,
                                accelerationOverride,
                                centripetalAccelerationOverride, false);
                mp2 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
                                List.of(new Pose2d(118, 40, Rotation2d.fromDegrees(45)),
                                                new Pose2d(190, 80, new Rotation2d()),
                                                new CirclePath(new Translation2d(218, 114), 38,
                                                                Rotation2d.fromDegrees(-90),
                                                                Rotation2d.fromDegrees(-135),
                                                                false)),
                                Double.MAX_VALUE, false, false, new ArrayList<>(), velocityOverride,
                                accelerationOverride,
                                centripetalAccelerationOverride, false);
                mp3 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
                                List.of(new Pose2d(213, 135, Rotation2d.fromDegrees(-110)),
                                                new Pose2d(305, 63, new Rotation2d())),
                                Double.MAX_VALUE, false, false, new ArrayList<>(), velocityOverride,
                                accelerationOverride,
                                centripetalAccelerationOverride, false);
                mp4 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
                                List.of(new Pose2d(270, 40, Rotation2d.fromDegrees(-35)),
                                                new CirclePath(new Translation2d(304, 64), 28.5,
                                                                Rotation2d.fromDegrees(-90),
                                                                Rotation2d.fromDegrees(90),
                                                                false)),
                                Double.MAX_VALUE, false, false, new ArrayList<>(), velocityOverride,
                                accelerationOverride,
                                centripetalAccelerationOverride, false);
                mp5 = new NewRunMotionProfile(driveTrain, odometry, velocityOverride,
                                List.of(new Pose2d(315, 82, Rotation2d.fromDegrees(165)),
                                                new Pose2d(240, 80, Rotation2d.fromDegrees(180)),
                                                new Pose2d(35, 60, Rotation2d.fromDegrees(180))),
                                Double.MAX_VALUE, false, false, new ArrayList<>(), velocityOverride,
                                accelerationOverride,
                                centripetalAccelerationOverride, false);
                // Add your addCommands(new FooCommand(), new BarCommand());
                if (odometry != null && driveTrain != null) {
                        addCommands(new SequentialCommandGroup(
                                        new InstantCommand(() -> odometry
                                                        .setPosition(new Pose2d(41.5, 90, new Rotation2d()))),
                                        mp1,
                                        new InstantCommand(() -> odometry.setPosition(new Translation2d(118, 40))), mp2,
                                        new InstantCommand(() -> odometry.setPosition(new Translation2d(213, 135))),
                                        mp3,
                                        new InstantCommand(() -> odometry.setPosition(new Translation2d(270, 40))), mp4,
                                        new InstantCommand(() -> odometry.setPosition(new Translation2d(315, 82))), mp5,
                                        new InstantCommand(() -> driveTrain.stop())), new RequireCommand(odometry));
                }
        }

        public static void main(String[] args) {
                Constants.setRobot(RobotType.ROBOT_2020);
                RunAutoNavBarrelRacing cmd = new RunAutoNavBarrelRacing(null, null);
                NewRunMotionProfile.runVisualizer(
                                List.of(cmd.mp1.visualizerGetTrajectory(), cmd.mp2.visualizerGetTrajectory(),
                                                cmd.mp3.visualizerGetTrajectory(),
                                                cmd.mp4.visualizerGetTrajectory(), cmd.mp5.visualizerGetTrajectory()),
                                cmd.mp4.visualizerGetTrackWidth(), 80,
                                List.of(new TrajectoryMarker(new Translation2d(30, 120), markerDiameter,
                                                markerColorStart),
                                                new TrajectoryMarker(new Translation2d(60, 120), markerDiameter,
                                                                markerColorStart),
                                                new TrajectoryMarker(new Translation2d(30, 60), markerDiameter,
                                                                markerColorStart),
                                                new TrajectoryMarker(new Translation2d(60, 60), markerDiameter,
                                                                markerColorStart),
                                                new TrajectoryMarker(new Translation2d(150, 60), markerDiameter,
                                                                markerColorMain),
                                                new TrajectoryMarker(new Translation2d(240, 120), markerDiameter,
                                                                markerColorMain),
                                                new TrajectoryMarker(new Translation2d(300, 60), markerDiameter,
                                                                markerColorMain)));
        }
}
