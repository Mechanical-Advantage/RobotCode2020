/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainBase;

public class RunMotionProfile extends SequentialCommandGroup {

  // These two values were found by converting suggested values of 2/0.7 from
  // meters to inches, which may be completely wrong.
  private static final double kRamseteB = 2; // 79
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

  /**
   * Creates a new RunMotionProfile.
   */
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
      maxAcceleration = 75; // Guess
    }
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    driveOdometry = new DifferentialDriveOdometry(initialPosition.getRotation(), initialPosition);
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, maxVoltage);
    TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveKinematics)
        .addConstraint(voltageConstraint).setStartVelocity(initialVelocity).setEndVelocity(endVelocity)
        .setReversed(reversed);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(initialPosition, intermediatePoints, endPosition,
        config);
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
    driveOdometry.update(Rotation2d.fromDegrees(ahrs.getYaw()), driveTrain.getDistanceLeft(),
        driveTrain.getDistanceRight());
    return driveOdometry.getPoseMeters();
  }
}
