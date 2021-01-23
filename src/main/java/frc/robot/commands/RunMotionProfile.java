/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;

public class RunMotionProfile extends CommandBase {

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
  private RobotOdometry odometry;
  private boolean dynamicTrajectory;
  private boolean relativeTrajectory;
  private DifferentialDriveKinematics driveKinematics;
  private Trajectory trajectory;
  private Trajectory baseTrajectory; // Trajectory before running relativeTo
  private double startTime;
  private MPGenerator generator;
  private List<Translation2d> intermediatePoints;
  private Pose2d endPosition;
  private TrajectoryConfig config;
  private boolean trajectoryUpdated;
  private boolean followerStarted;
  private RamseteCommand followerCommand;

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using the
   * intuitive coorindate system.
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param initialPosition    The starting pose for the profile
   * @param initialVelocity    The velocity at the beginning of the profile
   * @param intermediatePoints The points in between the start and end of the
   *                           profile (translation only)
   * @param endPosition        The end pose
   * @param endVelocity        The target velocity at the end of the profile
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   * 
   */
  public RunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative) {
    // The other constructor's relative trajectory handling is unneccessary with a
    // defined start point so always pass false and do the other neccessary logic
    // here
    this(driveTrain, odometry, initialPosition, initialVelocity, intermediatePoints, endPosition, endVelocity, reversed,
        relative, true);
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, with an
   * option to use the WPILib coorindate system.
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param initialPosition    The starting pose for the profile
   * @param initialVelocity    The velocity at the beginning of the profile
   * @param intermediatePoints The points in between the start and end of the
   *                           profile (translation only)
   * @param endPosition        The end pose
   * @param endVelocity        The target velocity at the end of the profile
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   * @param useIntuitive       Whether the pose inputs are in the intuitive system
   * 
   */
  public RunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative, boolean useIntuitive) {
    // The other constructor's relative trajectory handling is unneccessary with a
    // defined start point so always pass false and do the other neccessary logic
    // here
    this(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, false, useIntuitive);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(useIntuitive ? convertPose(initialPosition) : initialPosition, initialVelocity);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using the intuitive coorindate system.
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param intermediatePoints The points in between the start and end of the
   *                           profile (translation only)
   * @param endPosition        The end pose
   * @param endVelocity        The target velocity at the end of the profile
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   */
  public RunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative) {
    this(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, relative, true);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * with an option to use the WPILib coorindate system.
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param intermediatePoints The points in between the start and end of the
   *                           profile (translation only)
   * @param endPosition        The end pose
   * @param endVelocity        The target velocity at the end of the profile
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   * @param useIntuitive       Whether the pose inputs are in the intuitive system
   */
  @SuppressWarnings("incomplete-switch")
  public RunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative, boolean useIntuitive) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    this.odometry = odometry;
    dynamicTrajectory = true;
    switch (Constants.getRobot()) {
      case ROBOT_2019:
        kS = 1.21;
        kV = 0.0591;
        kA = 0.0182;
        trackWidth = 27.5932064868814;
        maxVelocity = 150;
        maxAcceleration = 50;
        break;
      case ROBOT_2020:
      case ROBOT_2020_DRIVE:
        kS = 0.14;
        kV = 0.0758;
        kA = 0.0128;
        trackWidth = 24.890470780033485;
        maxVelocity = 120;
        maxAcceleration = 50;
        break;
    }
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, maxVoltage);
    // Convert from intuitive to WPILib
    if (useIntuitive) {
      try {
        convertTranslationList(intermediatePoints);
      } catch (UnsupportedOperationException e) {
        // Make the list modifiable and try again
        intermediatePoints = new ArrayList<Translation2d>(intermediatePoints);
        convertTranslationList(intermediatePoints);
      }
    }
    this.intermediatePoints = intermediatePoints;
    this.endPosition = useIntuitive ? convertPose(endPosition) : endPosition;
    config = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveKinematics)
        .addConstraint(voltageConstraint).setEndVelocity(endVelocity).setReversed(reversed);
    if (relative) {
      relativeTrajectory = true;
      dynamicTrajectory = false;
      startGeneration(new Pose2d(), 0);
    }
  }

  @Override
  public void initialize() {
    if (dynamicTrajectory && !trajectoryUpdated) {
      startGeneration();
    }
  }

  @Override
  public void execute() {
    if (trajectory == null) {
      // This will just set trajectory to null again if not generated yet
      trajectory = generator.getTrajectory();
      baseTrajectory = trajectory;
    }
    if (trajectory != null && !followerStarted) {
      if (relativeTrajectory) {
        Transform2d transform = odometry.getCurrentPose().minus(baseTrajectory.getInitialPose());
        // For this use case, transformBy is correct, not relativeTo
        trajectory = baseTrajectory.transformBy(transform);
        followerCommand = null;
      }
      startProfile();
    }

    if (Constants.tuningMode && followerStarted) {
      Pose2d pose = odometry.getCurrentPose();
      Pose2d currentPose = trajectory.sample(Timer.getFPGATimestamp() - startTime).poseMeters;
      Translation2d currentTranslation = currentPose.getTranslation();
      SmartDashboard.putNumber("MP/PosError", pose.getTranslation().getDistance(currentTranslation));
      SmartDashboard.putNumber("MP/PoseXError", pose.getTranslation().getX() - currentTranslation.getX());
      SmartDashboard.putNumber("MP/PoseYError", pose.getTranslation().getY() - currentTranslation.getY());
      SmartDashboard.putNumber("MP/AngleError", pose.getRotation().minus(currentPose.getRotation()).getDegrees());
    }
  }

  @Override
  public boolean isFinished() {
    return followerStarted && followerCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (followerStarted) {
      followerCommand.cancel();
    }
    if (dynamicTrajectory) {
      trajectoryUpdated = false;
    }
    followerStarted = false;
  }

  /**
   * Start generating the profile using the robot's current position. The next
   * time the command is run, it will use the pre-generated profile. This does
   * nothing if the profile has a fixed initial position.
   */
  public void startGeneration() {
    if (dynamicTrajectory) {
      startGeneration(odometry.getCurrentPose(), (driveTrain.getVelocityLeft() + driveTrain.getVelocityRight()) / 2);
    }
  }

  /**
   * Start generating the trajectory using the specified starting
   * position/velocity.
   * 
   * @param initialPosition The WPILib convention starting location
   * @param initialVelocity The initial velocity of the profile
   */
  private void startGeneration(Pose2d initialPosition, double initialVelocity) {
    trajectory = null;
    followerCommand = null; // The old command can't be reused if the profile is changed.
    config.setStartVelocity(initialVelocity);
    generator = new MPGenerator(initialPosition, intermediatePoints, endPosition, config);
    generator.start();
    trajectoryUpdated = true;
  }

  /**
   * Start the profile follower command.
   */
  private void startProfile() {
    if (followerCommand == null) {
      followerCommand = new RamseteCommand(trajectory, odometry::getCurrentPose,
          new RamseteController(kRamseteB, kRamseteZeta), driveKinematics, driveTrain::driveInchesPerSec);
    }
    followerCommand.schedule();
    followerStarted = true;
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Converts a pose from Y-positive=forward, X-positive=right (intuitive) to
   * X-positive=forward, Y-positive=left (WPILib) or vise versa
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

  private static class MPGenerator extends Thread {

    private Pose2d initialPosition;
    private List<Translation2d> intermediatePoints;
    private Pose2d endPosition;
    private TrajectoryConfig config;
    private volatile Trajectory trajectory;

    public MPGenerator(Pose2d initialPosition, List<Translation2d> intermediatePoints, Pose2d endPosition,
        TrajectoryConfig config) {
      this.intermediatePoints = intermediatePoints;
      this.initialPosition = initialPosition;
      this.endPosition = endPosition;
      this.config = config;
    }

    @Override
    public void run() {
      trajectory = TrajectoryGenerator.generateTrajectory(initialPosition, intermediatePoints, endPosition, config);
    }

    @Override
    public void start() {
      trajectory = null;
      super.start();
    }

    public Trajectory getTrajectory() {
      return trajectory;
    }
  }
}
