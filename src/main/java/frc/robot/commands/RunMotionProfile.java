/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;

public class RunMotionProfile extends SequentialCommandGroup {

  private static final double kRamseteB = 0.0025; // 0.05 seems to be equivalent to the recommendation for meters
  private static final double kRamseteZeta = 0.7;
  private static final double maxVoltage = 10; // WPILib docs suggest less than 12 because of voltage drop

  private double kS; // Volts
  private double kV; // Volt seconds per inch
  private double kA; // Volt seconds squared per inch
  private double trackWidth;
  private double maxVelocity; // in/s
  private double maxAcceleration; // in/s^2

  private DriveTrainBase driveTrain;
  private AHRS ahrs;
  private DifferentialDriveKinematics driveKinematics;
  private DifferentialDriveOdometry driveOdometry;
  private Trajectory trajectory;
  private double initialDistanceLeft;
  private double initialDistanceRight;
  private double startTime;

  /**
   * Creates a new RunMotionProfile.
   */
  @SuppressWarnings("incomplete-switch")
  public RunMotionProfile(DriveTrainBase driveTrain, AHRS ahrs, Pose2d initialPosition, double initialVelocity,
      List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity, boolean reversed) {
    this.driveTrain = driveTrain;
    this.ahrs = ahrs;
    switch (Constants.getRobot()) {
    case ROBOT_2019:
      kS = 1.21;
      kV = 0.0591;
      kA = 0.0182;
      trackWidth = 27.5932064868814;
      maxVelocity = 150;
      maxAcceleration = 50;
    }
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    driveOdometry = new DifferentialDriveOdometry(initialPosition.getRotation(), initialPosition);
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, maxVoltage);
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveKinematics)
        .addConstraint(voltageConstraint).setStartVelocity(initialVelocity).setEndVelocity(endVelocity)
        .setReversed(reversed);
    // Convert from intuitive to WPILib
    try {
      convertTranslationList(intermediatePoints);
    } catch (UnsupportedOperationException e) {
      // Make the list modifiable and try again
      intermediatePoints = new ArrayList<Translation2d>(intermediatePoints);
      convertTranslationList(intermediatePoints);
    }
    trajectory = TrajectoryGenerator.generateTrajectory(convertPose(initialPosition), intermediatePoints,
        convertPose(endPosition), config);
    addCommands(new InstantCommand(() -> {
      initialDistanceLeft = driveTrain.getDistanceLeft();
      initialDistanceRight = driveTrain.getDistanceRight();
      driveOdometry.resetPosition(initialPosition, Rotation2d.fromDegrees(ahrs.getYaw() * -1));
      startTime = Timer.getFPGATimestamp();
    }, driveTrain));
    addCommands(new RamseteCommand(trajectory, this::getCurrentPose, new RamseteController(kRamseteB, kRamseteZeta),
        driveKinematics, driveTrain::driveInchesPerSec, driveTrain));
  }

  /**
   * Gets the current pose of the robot relative to the initial position. Must be
   * called every cycle exactly once (but the RamseteCommand should do that).
   * 
   * @return The current pose
   */
  private Pose2d getCurrentPose() {
    // Odometry classes expect counterclockwise positive yaw
    driveOdometry.update(Rotation2d.fromDegrees(ahrs.getYaw() * -1), driveTrain.getDistanceLeft() - initialDistanceLeft,
        driveTrain.getDistanceRight() - initialDistanceRight);
    Pose2d pose = driveOdometry.getPoseMeters();
    if (Constants.tuningMode) {
      // Pose2d intuitivePose = convertPose(pose, true);
      SmartDashboard.putNumber("MP/PoseY", pose.getTranslation().getY());
      SmartDashboard.putNumber("MP/PoseX", pose.getTranslation().getX());
      SmartDashboard.putNumber("MP/PoseYaw", pose.getRotation().getDegrees());
      Pose2d currentPose = trajectory.sample(Timer.getFPGATimestamp() - startTime).poseMeters;
      Translation2d currentTranslation = currentPose.getTranslation();
      SmartDashboard.putNumber("MP/PosError", pose.getTranslation().getDistance(currentTranslation));
      SmartDashboard.putNumber("MP/PoseXError", pose.getTranslation().getX() - currentTranslation.getX());
      SmartDashboard.putNumber("MP/PoseYError", pose.getTranslation().getY() - currentTranslation.getY());
      SmartDashboard.putNumber("MP/AngleError", pose.getRotation().minus(currentPose.getRotation()).getDegrees());
    }
    return pose;
  }

  /**
   * Converts a pose from Y-positive=forward, X-positive=right to
   * X-positive=forward, Y-positive=left or vise versa
   * 
   * @param input The input pose
   * @return The transformed pose
   */
  private Pose2d convertPose(Pose2d input) {
    return new Pose2d(new Translation2d(input.getTranslation().getY(), input.getTranslation().getX() * -1),
        input.getRotation().times(-1));
  }

  /**
   * Converts a list of Translation2d from intuitive to WPILib in place
   * 
   * @param input The list to modify
   */
  private void convertTranslationList(List<Translation2d> input) {
    input.replaceAll(point -> point.rotateBy(Rotation2d.fromDegrees(90)));
  }
}
