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
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.trajectory.CirclePath;
import frc.robot.util.trajectory.CirclePathConstraint;
import frc.robot.util.trajectory.InchesToMetersConstraint;
import frckit.tools.pathview.TrajectoryMarker;
import frckit.tools.pathview.TrajectoryVisualizer;
import frckit.util.GeomUtil;

public class NewRunMotionProfile extends CommandBase {

  private static final double kRamseteB = 2; // 0.05 seems to be equivalent to the recommendation for meters
  private static final double kRamseteZeta = 0.7;
  private static final double maxVoltage = 10; // WPILib docs suggest less than 12 because of voltage drop

  private double kS; // Volts
  private double kV; // Volt seconds per meter
  private double kA; // Volt seconds squared per meter
  private double trackWidth;
  private double maxVelocity; // m/s
  private double maxAcceleration; // m/s^2
  private double maxCentripetalAcceleration; // m/s^2

  private DriveTrainBase driveTrain;
  private RobotOdometry odometry;
  private boolean dynamicTrajectory;
  private boolean relativeTrajectory;
  private DifferentialDriveKinematics driveKinematics;
  private Trajectory trajectory;
  private Trajectory baseTrajectory; // Trajectory before running relativeTo
  private double startTime;
  private MPGenerator generator;
  private List<Pose2d> waypointPoses; // Does not include initial position
  private List<CirclePath> circlePaths = new ArrayList<>();
  private List<Translation2d> intermediatePointsTranslations;
  private Pose2d endPosition;
  private boolean useQuintic = false;
  private TrajectoryConfig config;
  private boolean trajectoryUpdated;
  private boolean followerStarted;
  private RamseteCommand followerCommand;

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * quintic spline (includes constraint overrides)
   * 
   * @param driveTrain                      The drive train
   * @param odometry                        The robot odometry
   * @param initialVelocity                 The velocity at the beginning of the
   *                                        profile (in/s)
   * @param waypointData                    The poses & circle paths including
   *                                        initial position (in)
   * @param endVelocity                     The target velocity at the end of the
   *                                        profile (in/s)
   * @param reversed                        Whether the robot drives backwards
   *                                        during the profile
   * @param relative                        Whether the profile is a relative
   *                                        change to the robot position as
   *                                        opposed to field coordinates
   * @param extraConstraints                Extra constrints to apply to the
   *                                        trajectory
   * @param velocityOverride                Max velocity (inches/second) to be
   *                                        used instead of the default
   * @param accelerationOverride            Max acceleration (inches/second^2) to
   *                                        be used instead of the default
   * @param centripetalAccelerationOverride Max centripetal acceleration
   *                                        (inches/second^2) to be used instead
   *                                        of the default
   * @param includeVoltageConstraint        Whether to include the default
   *                                        differential voltage constraint
   *                                        (disabling this is sometimes useful
   *                                        when overriding acceleration)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, double initialVelocity,
      List<Object> waypointData, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints, Double velocityOverride, Double accelerationOverride,
      Double centripetalAccelerationOverride, boolean includeVoltageConstraint) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData, reversed);
    Pose2d initialPosition = this.waypointPoses.remove(0);
    useQuintic = true;

    // Identical to constructor for fixed position & cubic
    setup(driveTrain, odometry, null, null, Units.inchesToMeters(endVelocity), reversed, false, extraConstraints,
        inchesToMetersIfNotNull(velocityOverride), inchesToMetersIfNotNull(accelerationOverride),
        inchesToMetersIfNotNull(centripetalAccelerationOverride), includeVoltageConstraint);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(initialPosition, Units.inchesToMeters(initialVelocity));
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * quintic spline
   * 
   * @param driveTrain       The drive train
   * @param odometry         The robot odometry
   * @param initialVelocity  The velocity at the beginning of the profile (in/s)
   * @param waypointData     The poses & circle paths including initial position
   *                         (in)
   * @param endVelocity      The target velocity at the end of the profile (in/s)
   * @param reversed         Whether the robot drives backwards during the profile
   * @param relative         Whether the profile is a relative change to the robot
   *                         position as opposed to field coordinates
   * @param extraConstraints Extra constrints to apply to the trajectory
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, double initialVelocity,
      List<Object> waypointData, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints) {
    this(driveTrain, odometry, initialVelocity, waypointData, endVelocity, reversed, relative, extraConstraints, null,
        null, null, true);
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * cubic spline (includes constraint overrides)
   * 
   * @param driveTrain                      The drive train
   * @param odometry                        The robot odometry
   * @param initialPosition                 The starting pose for the profile (in)
   * @param initialVelocity                 The velocity at the beginning of the
   *                                        profile (in/s)
   * @param intermediatePoints              The points in between the start and
   *                                        end of the profile, translation only
   *                                        (in)
   * @param endPosition                     The end pose (in)
   * @param endVelocity                     The target velocity at the end of the
   *                                        profile (in/s)
   * @param reversed                        Whether the robot drives backwards
   *                                        during the profile
   * @param relative                        Whether the profile is a relative
   *                                        change to the robot position as
   *                                        opposed to field coordinates
   * @param extraConstraints                Extra constrints to apply to the
   *                                        trajectory
   * @param velocityOverride                Max velocity (inches/second) to be
   *                                        used instead of the default
   * @param accelerationOverride            Max acceleration (inches/second^2) to
   *                                        be used instead of the default
   * @param centripetalAccelerationOverride Max centripetal acceleration
   *                                        (inches/second^2) to be used instead
   *                                        of the default
   * @param includeVoltageConstraint        Whether to include the default
   *                                        differential voltage constraint
   *                                        (disabling this is sometimes useful
   *                                        when overriding acceleration)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints, Double velocityOverride,
      Double accelerationOverride, Double centripetalAccelerationOverride, boolean includeVoltageConstraint) {
    // The setup function's relative trajectory handling is unneccessary with a
    // defined start point so always pass false and do the other neccessary logic
    // here
    updateConstants();
    setup(driveTrain, odometry, convertTranslationListToMeters(intermediatePoints),
        GeomUtil.inchesToMeters(endPosition), Units.inchesToMeters(endVelocity), reversed, false, extraConstraints,
        inchesToMetersIfNotNull(velocityOverride), inchesToMetersIfNotNull(accelerationOverride),
        inchesToMetersIfNotNull(centripetalAccelerationOverride), includeVoltageConstraint);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(GeomUtil.inchesToMeters(initialPosition), Units.inchesToMeters(initialVelocity));
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * cubic spline
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param initialPosition    The starting pose for the profile (in)
   * @param initialVelocity    The velocity at the beginning of the profile (in/s)
   * @param intermediatePoints The points in between the start and end of the
   *                           profile, translation only (in)
   * @param endPosition        The end pose (in)
   * @param endVelocity        The target velocity at the end of the profile
   *                           (in/s)
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   * @param extraConstraints   Extra constrints to apply to the trajectory
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints) {
    this(driveTrain, odometry, initialPosition, initialVelocity, intermediatePoints, endPosition, endVelocity, reversed,
        relative, extraConstraints, null, null, null, true);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a quintic spline (includes constraint overrides)
   * 
   * @param driveTrain                      The drive train
   * @param odometry                        The robot odometry
   * @param waypointData                    The poses & circle paths after initial
   *                                        position (in)
   * @param endVelocity                     The target velocity at the end of the
   *                                        profile (in/s)
   * @param reversed                        Whether the robot drives backwards
   *                                        during theprofile
   * @param relative                        Whether the profile is a relative
   *                                        change to the robot position as
   *                                        opposed to field coordinates
   * @param extraConstraints                Extra constrints to apply to the
   *                                        trajectory
   * @param velocityOverride                Max velocity (inches/second) to be
   *                                        used instead of the default
   * @param accelerationOverride            Max acceleration (inches/second^2) to
   *                                        be used instead of the default
   * @param centripetalAccelerationOverride Max centripetal acceleration
   *                                        (inches/second^2) to be used instead
   *                                        of the default
   * @param includeVoltageConstraint        Whether to include the default
   *                                        differential voltage constraint
   *                                        (disabling this is sometimes useful
   *                                        when overriding acceleration)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Object> waypointData,
      double endVelocity, boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints,
      Double velocityOverride, Double accelerationOverride, Double centripetalAccelerationOverride,
      boolean includeVoltageConstraint) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData, reversed);
    useQuintic = true;
    setup(driveTrain, odometry, null, null, Units.inchesToMeters(endVelocity), reversed, relative, extraConstraints,
        inchesToMetersIfNotNull(velocityOverride), inchesToMetersIfNotNull(accelerationOverride),
        inchesToMetersIfNotNull(centripetalAccelerationOverride), includeVoltageConstraint);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a quintic spline
   * 
   * @param driveTrain       The drive train
   * @param odometry         The robot odometry
   * @param waypointData     The poses & circle paths after initial position (in)
   * @param endVelocity      The target velocity at the end of the profile (in/s)
   * @param reversed         Whether the robot drives backwards during theprofile
   * @param relative         Whether the profile is a relative change to the robot
   *                         position as opposed to field coordinates
   * @param extraConstraints Extra constrints to apply to the trajectory
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Object> waypointData,
      double endVelocity, boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints) {
    this(driveTrain, odometry, waypointData, endVelocity, reversed, relative, extraConstraints, null, null, null, true);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a cubic spline (includes constraint overrides)
   * 
   * @param driveTrain                      The drive train
   * @param odometry                        The robot odometry
   * @param intermediatePoints              The points in between the start and
   *                                        end of the profile, translation only
   *                                        (in)
   * @param endPosition                     The end pose (in)
   * @param endVelocity                     The target velocity at the end of the
   *                                        profile (in/s)
   * @param reversed                        Whether the robot drives backwards
   *                                        during the profile
   * @param relative                        Whether the profile is a relative
   *                                        change to the robot position as
   *                                        opposed to field coordinates
   * @param extraConstraints                Extra constrints to apply to the
   *                                        trajectory
   * @param velocityOverride                Max velocity (inches/second) to be
   *                                        used instead of the default
   * @param accelerationOverride            Max acceleration (inches/second^2) to
   *                                        be used instead of the default
   * @param centripetalAccelerationOverride Max centripetal acceleration
   *                                        (inches/second^2) to be used instead
   *                                        of the default
   * @param includeVoltageConstraint        Whether to include the default
   *                                        differential voltage constraint
   *                                        (disabling this is sometimes useful
   *                                        when overriding acceleration)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints, Double velocityOverride, Double accelerationOverride,
      Double centripetalAccelerationOverride, boolean includeVoltageConstraint) {
    updateConstants();
    setup(driveTrain, odometry, convertTranslationListToMeters(intermediatePoints),
        GeomUtil.inchesToMeters(endPosition), Units.inchesToMeters(endVelocity), reversed, relative, extraConstraints,
        inchesToMetersIfNotNull(velocityOverride), inchesToMetersIfNotNull(accelerationOverride),
        inchesToMetersIfNotNull(centripetalAccelerationOverride), includeVoltageConstraint);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a cubic spline
   * 
   * @param driveTrain         The drive train
   * @param odometry           The robot odometry
   * @param intermediatePoints The points in between the start and end of the
   *                           profile, translation only (in)
   * @param endPosition        The end pose (in)
   * @param endVelocity        The target velocity at the end of the profile
   *                           (in/s)
   * @param reversed           Whether the robot drives backwards during the
   *                           profile
   * @param relative           Whether the profile is a relative change to the
   *                           robot position as opposed to field coordinates
   * @param extraConstraints   Extra constrints to apply to the trajectory (must
   *                           be metric)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints) {
    this(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, relative, extraConstraints, null,
        null, null, true);
  }

  /**
   * Updates constants based on current robot
   */
  @SuppressWarnings("incomplete-switch")
  private void updateConstants() {

    // All constants defined in inches
    switch (Constants.getRobot()) {
      case ROBOT_2019:
        kS = 1.21;
        kV = 0.0591;
        kA = 0.0182;
        trackWidth = 27.5932064868814;
        maxVelocity = 150;
        maxAcceleration = 50;
        maxCentripetalAcceleration = 200;
        break;
      case ROBOT_2020_DRIVE:
        kS = 0.14;
        kV = 0.0758;
        kA = 0.0128;
        trackWidth = 24.890470780033485;
        maxVelocity = 120;
        maxAcceleration = 50;
        maxCentripetalAcceleration = 200;
        break;
      case ROBOT_2020:
        // Data from blue wheels:
        // kS = 0.124;
        // kV = 0.0722;
        // kA = 0.00475;
        kS = 0.165;
        kV = 0.0801;
        kA = 0.0118;
        trackWidth = 25.934;
        maxVelocity = 130;
        maxAcceleration = 130;
        maxCentripetalAcceleration = 120;
        break;
    }

    // Convert to meters
    kV = Units.metersToInches(kV);
    kA = Units.metersToInches(kA);
    trackWidth = Units.inchesToMeters(trackWidth);
    maxVelocity = Units.inchesToMeters(maxVelocity);
    maxAcceleration = Units.inchesToMeters(maxAcceleration);
    maxCentripetalAcceleration = Units.inchesToMeters(maxCentripetalAcceleration);
  }

  /**
   * Sets up a new RunMotionProfile that starts from the robot's current position,
   * using a cubic spline.
   * 
   * @param driveTrain                      The drive train
   * @param odometry                        The robot odometry
   * @param intermediatePoints              The points in between the start and
   *                                        end of the profile (translation only)
   * @param endPosition                     The end pose
   * @param endVelocity                     The target velocity at the end of the
   *                                        profile
   * @param reversed                        Whether the robot drives backwards
   *                                        during the profile
   * @param relative                        Whether the profile is a relative
   *                                        change to the robot position as
   *                                        opposed to field coordinates
   * @param extraConstraints                Extra constrints to apply to the
   *                                        trajectory
   * @param centripetalAccelerationOverride Max centripetal acceleration
   *                                        (meters/second^2) to be used instead
   *                                        of the default. Negative to use
   *                                        default.
   */
  private void setup(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints, Double velocityOverride, Double accelerationOverride,
      Double centripetalAccelerationOverride, boolean includeVoltageConstraint) {
    this.driveTrain = driveTrain;
    if (driveTrain != null) {
      addRequirements(driveTrain);
    }
    this.odometry = odometry;
    this.intermediatePointsTranslations = intermediatePoints;
    this.endPosition = endPosition;
    dynamicTrajectory = true;

    // Set up basic contraints
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, maxVoltage);
    CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(
        centripetalAccelerationOverride == null ? maxCentripetalAcceleration : centripetalAccelerationOverride);
    config = new TrajectoryConfig((velocityOverride == null ? maxVelocity : velocityOverride),
        (accelerationOverride == null ? maxAcceleration : accelerationOverride)).setKinematics(driveKinematics)
            .addConstraint(centripetalAccelerationConstraint).setEndVelocity(endVelocity).setReversed(reversed);
    if (includeVoltageConstraint) {
      config.addConstraint(voltageConstraint);
    }
    for (int i = 0; i < extraConstraints.size(); i++) {
      config.addConstraint(new InchesToMetersConstraint(extraConstraints.get(i)));
    }

    // Apply circle constraints
    List<TrajectoryConstraint> constraints = config.getConstraints(); // Retrieving constraints from the config ensures
                                                                      // the kinematics constraint is included
    List<TrajectoryConstraint> circleConstraints = new ArrayList<>();
    for (int circleIndex = 0; circleIndex < circlePaths.size(); circleIndex++) {
      for (int constraintIndex = 0; constraintIndex < constraints.size(); constraintIndex++) {
        circleConstraints.add(new CirclePathConstraint(circlePaths.get(circleIndex), constraints.get(constraintIndex)));
      }
    }
    config.addConstraints(circleConstraints);

    // Start generation
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
      if (trajectory != null) {
        trajectory = adjustCircleTrajectories(trajectory);
      }
      baseTrajectory = trajectory;
    }
    if (trajectory != null && !followerStarted) {
      if (relativeTrajectory) {
        Transform2d transform = getCurrentPoseMeters().minus(baseTrajectory.getInitialPose());
        // For this use case, transformBy is correct, not relativeTo
        trajectory = baseTrajectory.transformBy(transform);
      }
      startProfile();
    }

    if (Constants.tuningMode && followerStarted) {
      Pose2d pose = getCurrentPoseMeters();
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
      startGeneration(getCurrentPoseMeters(),
          Units.inchesToMeters((driveTrain.getVelocityLeft() + driveTrain.getVelocityRight()) / 2));
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
    config.setStartVelocity(initialVelocity);
    if (useQuintic) {
      List<Pose2d> fullWaypointPoses = new ArrayList<>();
      fullWaypointPoses.add(initialPosition);
      fullWaypointPoses.addAll(waypointPoses);
      generator = new MPGenerator(fullWaypointPoses, config);
    } else {
      generator = new MPGenerator(initialPosition, intermediatePointsTranslations, endPosition, config);
    }
    generator.start();
    trajectoryUpdated = true;
  }

  /**
   * Start the profile follower command.
   */
  private void startProfile() {
    followerCommand = new RamseteCommand(trajectory, this::getCurrentPoseMeters,
        new RamseteController(kRamseteB, kRamseteZeta), driveKinematics, this::driveMetersPerSecond);
    followerCommand.schedule();
    followerStarted = true;
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Retrieves the current position from odometry in meters
   */
  private Pose2d getCurrentPoseMeters() {
    return GeomUtil.inchesToMeters(odometry.getCurrentPose());
  }

  /**
   * Drives at meters per second (converts meters to inches)
   */
  private void driveMetersPerSecond(double left, double right) {
    driveTrain.driveInchesPerSec(Units.metersToInches(left), Units.metersToInches(right));
  }

  /**
   * Retrieves the current trajectory for use in the visualizer. This should not
   * be called from robot code, but instead used only when testing on development
   * machines. It can be used by putting a main method in the command file of the
   * profile, and then running that main method from within VSCode - this ensures
   * it is never called on the robot on accident.
   *
   * This version of the method should be used for profiles that have a starting
   * position which is determined by the current position of the robot. Since
   * there is no real robot in the visualization, a fake initial robot position
   * must be provided
   * 
   * @param initialRobotPosition The starting position of the robot to test with
   */
  public Trajectory visualizerGetTrajectory(Pose2d initialRobotPosition) {
    if (initialRobotPosition != null) {
      startGeneration(GeomUtil.inchesToMeters(initialRobotPosition), 0.0);
    }
    // Busy-wait for trajectory to finish generating
    Trajectory trajectory = null;
    while (trajectory == null) {
      try {
        Thread.sleep(10); // Blocking busy-wait
      } catch (InterruptedException e) {
        return new Trajectory(); // We got interrupted (probably user hit stop), so just exit
      }
      trajectory = generator.getTrajectory(); // Attempt to grab new path
    }
    return adjustCircleTrajectories(trajectory);
  }

  /**
   * Retrieves the current trajectory for use in the visualizer. This should not
   * be called from robot code, but instead used only when testing on development
   * machines. It can be used by putting a main method in the command file of the
   * profile, and then running that main method from within VSCode - this ensures
   * it is never called on the robot on accident.
   *
   * This version of the method should be used for profiles that have a defined
   * starting position, i.e. not in dynamic mode.
   * 
   * @param ppm           The number of pixels which should represent one meter.
   *                      80 is a good starting value
   * @param markersInches A list of markers to draw on the path (inches).
   */
  public Trajectory visualizerGetTrajectory() {
    return visualizerGetTrajectory(null);
  }

  /**
   * Gets the track width for the current robot, which is necessary to run the
   * visualizer.
   * 
   * @return Track width in meters
   */
  public double visualizerGetTrackWidth() {
    return trackWidth;
  }

  /**
   * Runs the visualizer using a set of trajectories. This should not be called
   * from robot code, but instead used only when testing on development machines.
   * It can be used by putting a main method in the command file of the profile,
   * and then running that main method from within VSCode - this ensures it is
   * never called on the robot on accident.
   * 
   * @param trajectories  A list of trajectories to visualize (retrieve using
   *                      visualizerGetTrajectory)
   * @param trackWidth    Track width in meters (retrieve using
   *                      visualizerGetTrackWidth)
   * @param ppm           The number of pixels which should represent one meter.
   *                      80 is a good starting value
   * @param markersInches A list of markers to draw on the path (inches).
   */
  public static void runVisualizer(List<Trajectory> trajectories, double trackWidth, double ppm,
      List<TrajectoryMarker> markersInches) {
    List<TrajectoryMarker> markersMeters = new ArrayList<>();
    for (var i = 0; i < markersInches.size(); i++) {
      TrajectoryMarker marker = markersInches.get(i);
      markersMeters.add(new TrajectoryMarker(GeomUtil.inchesToMeters(marker.getPosition()),
          Units.inchesToMeters(marker.getDiameterMeters()), marker.getColor()));
    }
    TrajectoryVisualizer viz = new TrajectoryVisualizer(ppm, trajectories, trackWidth, markersMeters);
    viz.start();
  }

  /**
   * Converts a list of poses in inches to meters
   * 
   * @param poses
   * @return Poses in meters
   */
  private static List<Pose2d> convertPoseListToMeters(List<Pose2d> poses) {
    List<Pose2d> output = new ArrayList<>();
    for (var i = 0; i < poses.size(); i++) {
      output.add(GeomUtil.inchesToMeters(poses.get(i)));
    }
    return output;
  }

  /**
   * Converts a list of translations in inches to meters
   * 
   * @param translations
   * @return Translations in meters
   */
  private static List<Translation2d> convertTranslationListToMeters(List<Translation2d> translations) {
    List<Translation2d> output = new ArrayList<>();
    for (var i = 0; i < translations.size(); i++) {
      output.add(GeomUtil.inchesToMeters(translations.get(i)));
    }
    return output;
  }

  /**
   * Converts a Double from inches to meters, returning null if the input is null.
   */
  private static Double inchesToMetersIfNotNull(Double input) {
    if (input == null) {
      return null;
    } else {
      return Units.inchesToMeters(input);
    }
  }

  /**
   * Processes a list of Pose2d and CirclePath objects into only Pose2d objects,
   * including saving circle path objects. Converts from inches to meters.
   */
  public List<Pose2d> processWaypointData(List<Object> waypointData, boolean reversed) {
    List<Pose2d> outputPoses = new ArrayList<>();
    for (int i = 0; i < waypointData.size(); i++) {
      if (waypointData.get(i).getClass() == Pose2d.class) {
        outputPoses.add(GeomUtil.inchesToMeters((Pose2d) waypointData.get(i)));
      } else if (waypointData.get(i).getClass() == CirclePath.class) {
        CirclePath circle = (CirclePath) waypointData.get(i);
        circle.reversed = reversed;
        outputPoses.addAll(circle.calcPoses());
        circlePaths.add(circle);
      }
    }
    return outputPoses;
  }

  /**
   * Processes a trajectory to fix curvature of circular sections
   * 
   * @param trajectory The original trajectory
   * @return The new trajectory
   */
  public Trajectory adjustCircleTrajectories(Trajectory trajectory) {
    Trajectory newTrajectory = trajectory;
    for (var i = 0; i < circlePaths.size(); i++) {
      newTrajectory = circlePaths.get(i).adjustTrajectory(newTrajectory);
    }
    return newTrajectory;
  }

  private static class MPGenerator extends Thread {

    private Pose2d initialPosition;
    private List<Translation2d> intermediatePointsTranslations;
    private List<Pose2d> waypointPoses; // Includes initial position
    private boolean useQuintic;
    private Pose2d endPosition;
    private TrajectoryConfig config;
    private volatile Trajectory trajectory;

    public MPGenerator(List<Pose2d> waypointPoses, TrajectoryConfig config) {
      this.waypointPoses = waypointPoses;
      this.config = config;
      useQuintic = true;
    }

    public MPGenerator(Pose2d initialPosition, List<Translation2d> intermediatePoints, Pose2d endPosition,
        TrajectoryConfig config) {
      this.intermediatePointsTranslations = intermediatePoints;
      this.initialPosition = initialPosition;
      this.endPosition = endPosition;
      this.config = config;
      useQuintic = false;
    }

    @Override
    public void run() {
      if (useQuintic) {
        trajectory = TrajectoryGenerator.generateTrajectory(waypointPoses, config);
      } else {
        trajectory = TrajectoryGenerator.generateTrajectory(initialPosition, intermediatePointsTranslations,
            endPosition, config);
      }
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
