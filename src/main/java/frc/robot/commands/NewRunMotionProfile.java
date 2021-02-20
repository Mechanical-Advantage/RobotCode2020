/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import edu.wpi.first.wpilibj.Timer;
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
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.physics.drivetrain.follower.DrivetrainTrajectoryFollower;
import frckit.physics.drivetrain.follower.NonholonomicNonlinearFeedback;
import frckit.physics.state.RigidBodyState2d;
import frckit.tools.pathview.TrajectoryMarker;
import frckit.tools.pathview.TrajectoryVisualizer;
import frckit.util.GeomUtil;

public class NewRunMotionProfile extends CommandBase {
  private static final ExecutorService executor = Executors.newSingleThreadExecutor();

  private static final double kRamseteB = 2; // 0.05 seems to be equivalent to the recommendation for meters
  private static final double kRamseteZeta = 0.7;
  private static final double maxVoltage = 10; // WPILib docs suggest less than 12 because of voltage drop

  private double maxVelocity; // m/s
  private double maxAcceleration; // m/s^2
  private double maxCentripetalAcceleration; // m/s^2

  // Set default values when running in visualizer (TODO: retrieve these from the
  // drive always)
  private double kS = 0.11975;
  private double kV = 0.07719;
  private double kA = 0.01145;
  private double trackWidth = 25.934;

  private DriveTrainBase driveTrain;
  private RobotOdometry odometry;
  private boolean dynamicTrajectory;
  private boolean relativeTrajectory;
  private DifferentialDriveKinematics driveKinematics;
  private Trajectory trajectory;
  private Trajectory baseTrajectory; // Trajectory before running relativeTo
  private List<Pose2d> waypointPoses; // Does not include initial position
  private List<CirclePath> circlePaths = new ArrayList<>();
  private List<Translation2d> intermediatePointsTranslations;
  private Pose2d endPosition;
  private boolean useQuintic = false;
  private TrajectoryConfig config;
  private boolean trajectoryUpdated;
  private boolean followerStarted;
  private DrivetrainTrajectoryFollower follower;
  private Future<Trajectory> trajectoryFuture = null;

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
   * @param extraConstraints Extra constrints to apply to the trajectory (must be
   *                         metric)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, double initialVelocity,
      List<Object> waypointData, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData, reversed);
    Pose2d initialPosition = this.waypointPoses.remove(0);
    useQuintic = true;

    // Identical to constructor for fixed position & cubic
    setup(driveTrain, odometry, null, null, Units.inchesToMeters(endVelocity), reversed, false, extraConstraints);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(initialPosition, Units.inchesToMeters(initialVelocity));
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * quintic spline
   * 
   * @param driveTrain      The drive train
   * @param odometry        The robot odometry
   * @param initialVelocity The velocity at the beginning of the profile (in/s)
   * @param waypointData    The poses & circle paths including initial position
   *                        (in)
   * @param endVelocity     The target velocity at the end of the profile (in/s)
   * @param reversed        Whether the robot drives backwards during the profile
   * @param relative        Whether the profile is a relative change to the robot
   *                        position as opposed to field coordinates
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, double initialVelocity,
      List<Object> waypointData, double endVelocity, boolean reversed, boolean relative) {
    this(driveTrain, odometry, initialVelocity, waypointData, endVelocity, reversed, relative, new ArrayList<>());
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
   * @param extraConstraints   Extra constrints to apply to the trajectory (must
   *                           be metric)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints) {
    // The setup function's relative trajectory handling is unneccessary with a
    // defined start point so always pass false and do the other neccessary logic
    // here
    updateConstants();
    setup(driveTrain, odometry, convertTranslationListToMeters(intermediatePoints),
        GeomUtil.inchesToMeters(endPosition), Units.inchesToMeters(endVelocity), reversed, false, extraConstraints);
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
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative) {
    this(driveTrain, odometry, initialPosition, initialVelocity, intermediatePoints, endPosition, endVelocity, reversed,
        relative, new ArrayList<>());
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
   * @param extraConstraints Extra constrints to apply to the trajectory (must be
   *                         metric)
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Object> waypointData,
      double endVelocity, boolean reversed, boolean relative, List<TrajectoryConstraint> extraConstraints) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData, reversed);
    useQuintic = true;
    setup(driveTrain, odometry, null, null, Units.inchesToMeters(endVelocity), reversed, relative, extraConstraints);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a quintic spline
   * 
   * @param driveTrain   The drive train
   * @param odometry     The robot odometry
   * @param waypointData The poses & circle paths after initial position (in)
   * @param endVelocity  The target velocity at the end of the profile (in/s)
   * @param reversed     Whether the robot drives backwards during theprofile
   * @param relative     Whether the profile is a relative change to the robot
   *                     position as opposed to field coordinates
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Object> waypointData,
      double endVelocity, boolean reversed, boolean relative) {
    this(driveTrain, odometry, waypointData, endVelocity, reversed, relative, new ArrayList<>());
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
    updateConstants();
    setup(driveTrain, odometry, convertTranslationListToMeters(intermediatePoints),
        GeomUtil.inchesToMeters(endPosition), Units.inchesToMeters(endVelocity), reversed, relative, extraConstraints);
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
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative) {
    this(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, relative, new ArrayList<>());
  }

  /**
   * Updates constants based on current robot
   */
  @SuppressWarnings("incomplete-switch")
  private void updateConstants() {

    // All constants defined in inches
    switch (Constants.getRobot()) {
      case ROBOT_2019:
        maxVelocity = 150;
        maxAcceleration = 50;
        maxCentripetalAcceleration = 200;
        break;
      case ROBOT_2020_DRIVE:
        maxVelocity = 120;
        maxAcceleration = 50;
        maxCentripetalAcceleration = 200;
        break;
      case ROBOT_2020:
        maxVelocity = 130;
        maxAcceleration = 130;
        maxCentripetalAcceleration = 120;
        break;
    }

    // Convert to meters
    maxVelocity = Units.inchesToMeters(maxVelocity);
    maxAcceleration = Units.inchesToMeters(maxAcceleration);
    maxCentripetalAcceleration = Units.inchesToMeters(maxCentripetalAcceleration);
  }

  /**
   * Sets up a new RunMotionProfile that starts from the robot's current position,
   * using a cubic spline.
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
   * @param extraConstraints   Extra constrints to apply to the trajectory (must
   *                           be metric)
   */
  private void setup(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative,
      List<TrajectoryConstraint> extraConstraints) {
    this.driveTrain = driveTrain;
    if (driveTrain != null) {
      addRequirements(driveTrain);
      kS = driveTrain.getKs();
      kV = driveTrain.getKv();
      kA = driveTrain.getKa();
      trackWidth = driveTrain.getImpericalTrackWidth();
    }
    this.odometry = odometry;
    dynamicTrajectory = true;
    driveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, Units.metersToInches(kV), Units.metersToInches(kA)), driveKinematics,
        maxVoltage);
    CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(
        maxCentripetalAcceleration);
    this.intermediatePointsTranslations = intermediatePoints;
    this.endPosition = endPosition;
    config = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveKinematics)
        .addConstraint(voltageConstraint).addConstraint(centripetalAccelerationConstraint)
        .addConstraints(extraConstraints).setEndVelocity(endVelocity).setReversed(reversed);
    List<TrajectoryConstraint> allConstraints = new ArrayList<>();
    allConstraints.add(voltageConstraint);
    allConstraints.add(centripetalAccelerationConstraint);
    allConstraints.addAll(extraConstraints);
    for (int i = 0; i < circlePaths.size(); i++) {
      config.addConstraint(circlePaths.get(i).getConstraint(driveKinematics, maxVelocity, allConstraints));
    }
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
      if (trajectoryFuture != null && trajectoryFuture.isDone()) {
        try {
          trajectory = trajectoryFuture.get();
          trajectory = adjustCircleTrajectories(trajectory);
          baseTrajectory = trajectory;
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
    if (trajectory != null && !followerStarted) {
      if (relativeTrajectory) {
        Transform2d transform = getCurrentPoseMeters().minus(baseTrajectory.getInitialPose());
        // For this use case, transformBy is correct, not relativeTo
        trajectory = baseTrajectory.transformBy(transform);
      }
      follower = new DrivetrainTrajectoryFollower(trajectory, new NonholonomicNonlinearFeedback());
      followerStarted = true;
    }

    if (followerStarted) {
      RigidBodyState2d command = follower.update(Timer.getFPGATimestamp(), getCurrentPoseMeters());
      driveTrain.drive(command);
    }

    if (Constants.tuningMode && followerStarted) {
      Pose2d error = GeomUtil.metersToInches(follower.getError());
      SmartDashboard.putNumber("MP/PosError", error.getTranslation().getNorm());
      SmartDashboard.putNumber("MP/PoseXError", error.getTranslation().getX());
      SmartDashboard.putNumber("MP/PoseYError", error.getTranslation().getY());
      SmartDashboard.putNumber("MP/AngleError", error.getRotation().getDegrees());
    }
  }

  @Override
  public boolean isFinished() {
    return followerStarted && follower.isDone();
  }

  @Override
  public void end(boolean interrupted) {
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
      trajectoryFuture = executor.submit(() -> TrajectoryGenerator.generateTrajectory(fullWaypointPoses, config));
    } else {
      trajectoryFuture = executor.submit(() -> TrajectoryGenerator.generateTrajectory(initialPosition,
          intermediatePointsTranslations, endPosition, config));
    }
    trajectoryUpdated = true;
  }

  /**
   * Retrieves the current position from odometry in meters
   */
  private Pose2d getCurrentPoseMeters() {
    return GeomUtil.inchesToMeters(odometry.getCurrentPose());
  }

  /**
   * <<<<<<< HEAD ======= Drives at meters per second (converts meters to inches)
   */
  private void driveMetersPerSecond(double left, double right) {
    driveTrain.driveInchesPerSec(Units.metersToInches(left), Units.metersToInches(right));
  }

  /**
   * >>>>>>> atHomeAutos Runs the trajectory visualizer on this command. This
   * should not be called from robot code, but instead used only when testing on
   * development machines. It can be used by putting a main method in the command
   * file of the profile, and then running that main method from within VSCode -
   * this ensures it is never called on the robot on accident.
   *
   * This version of the method should be used for profiles that have a starting
   * position which is determined by the current position of the robot. Since
   * there is no real robot in the visualization, a fake initial robot position
   * must be provided
   * 
   * @param ppm                  The number of pixels which should represent one
   *                             meter. 80 is a good starting value
   * @param markersInches        A list of markers to draw on the path (inches).
   * @param initialRobotPosition The starting position of the robot to test with
   */
  public void visualize(double ppm, List<TrajectoryMarker> markersInches, Pose2d initialRobotPosition) {
    if (initialRobotPosition != null) {
      startGeneration(GeomUtil.inchesToMeters(initialRobotPosition), 0.0);
    }
    // Busy-wait for trajectory to finish generating
    Trajectory t = null;
    while (t == null) {
      try {
        Thread.sleep(10); // Blocking busy-wait
      } catch (InterruptedException e) {
        return; // We got interrupted (probably user hit stop), so just exit
      }
      if (trajectoryFuture != null && trajectoryFuture.isDone()) {
        try {
          t = trajectoryFuture.get();
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
    t = adjustCircleTrajectories(t);
    List<TrajectoryMarker> markersMeters = new ArrayList<>();
    for (var i = 0; i < markersInches.size(); i++) {
      TrajectoryMarker marker = markersInches.get(i);
      markersMeters.add(new TrajectoryMarker(GeomUtil.inchesToMeters(marker.getPosition()),
          Units.inchesToMeters(marker.getDiameterMeters()), marker.getColor()));
    }
    TrajectoryVisualizer viz = new TrajectoryVisualizer(ppm, List.of(t), trackWidth, markersMeters);
    viz.start();
  }

  /**
   * Runs the trajectory visualizer on this command. This should not be called
   * from robot code, but instead used only when testing on development machines.
   * It can be used by putting a main method in the command file of the profile,
   * and then running that main method from within VSCode - this ensures it is
   * never called on the robot on accident.
   *
   * This version of the method should be used for profiles that have a defined
   * starting position, i.e. not in dynamic mode.
   * 
   * @param ppm           The number of pixels which should represent one meter.
   *                      80 is a good starting value
   * @param markersInches A list of markers to draw on the path (inches).
   */
  public void visualize(double ppm, List<TrajectoryMarker> markersInches) {
    visualize(ppm, markersInches, null);
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

  /**
   * Represents a circular path to be used in a quintic spline
   **/
  public static class CirclePath {
    private static final double separationDistance = 0.00254; // Distance between points in meters (must be quite small
                                                              // to ensure continuous curve)

    public final Translation2d center;
    public final double radius;
    public final Rotation2d startingRotation;
    public final Rotation2d endingRotation;
    public final boolean clockwise;
    public boolean reversed = false;

    /**
     * Creates a circular path with the given properties
     * 
     * @param center           The center position of the circle (in)
     * @param radius           The radius of the circle (in)
     * @param startingRotation The rotation relative to the center at which to start
     *                         the path (NOT the starting rotation of the robot)
     * @param endingRotation   The rotation relative to the center at which to end
     *                         the path (NOT the ending rotation of the robot)
     * @param clockwise        Whether to move clockwise or countercloswise from the
     *                         start to end
     */
    public CirclePath(Translation2d center, double radius, Rotation2d startingRotation, Rotation2d endingRotation,
        boolean clockwise) {
      this.center = GeomUtil.inchesToMeters(center);
      this.radius = Units.inchesToMeters(radius);
      this.startingRotation = startingRotation;
      this.clockwise = clockwise;

      // Adjust ending rotation if full circle (simplifies other logic)
      if (startingRotation.getDegrees() == endingRotation.getDegrees()) {
        this.endingRotation = endingRotation.plus(Rotation2d.fromDegrees(clockwise ? 0.000001 : -0.000001));
      } else {
        this.endingRotation = endingRotation;
      }
    }

    /**
     * Calculates a series of points following the circumference of a circle, to be
     * used as waypoints for a quintic spline
     */
    public List<Pose2d> calcPoses() {
      Rotation2d separationAngle = Rotation2d.fromDegrees((separationDistance / (radius * 2 * Math.PI)) * 360);
      List<Pose2d> outputPoses = new ArrayList<>();
      Rotation2d currentRotation = startingRotation;
      boolean lastLeftOfEnd = currentRotation.minus(endingRotation).getDegrees() > 0;

      while (true) {
        // Calculate new pose for current rotation
        Transform2d transform = new Transform2d(new Translation2d(radius, 0),
            Rotation2d.fromDegrees((clockwise ? -90 : 90) * (reversed ? -1 : 1)));
        outputPoses.add(new Pose2d(center, currentRotation).transformBy(transform));
        if (clockwise) {
          currentRotation = currentRotation.minus(separationAngle);
        } else {
          currentRotation = currentRotation.plus(separationAngle);
        }

        // Determine if path is complete
        boolean leftOfEnd = currentRotation.minus(endingRotation).getDegrees() > 0;
        if (clockwise) {
          if (!leftOfEnd && lastLeftOfEnd) {
            break;
          }
        } else {
          if (leftOfEnd && !lastLeftOfEnd) {
            break;
          }
        }
        lastLeftOfEnd = leftOfEnd;
      }
      return outputPoses;
    }

    /**
     * Return a trajectory constraint for the path, given drive kinematics and other
     * constraints
     */
    public CirclePathConstraint getConstraint(DifferentialDriveKinematics driveKinematics, double maxVelocity,
        List<TrajectoryConstraint> constraints) {
      return new CirclePathConstraint(this, driveKinematics, maxVelocity, constraints);
    }

    /**
     * Processes a trajectory to fix curvature of the circular section
     */
    public Trajectory adjustTrajectory(Trajectory trajectory) {
      List<State> states = trajectory.getStates();
      for (var i = 0; i < states.size(); i++) {
        State currentState = states.get(i);
        if (contains(currentState.poseMeters)) {
          currentState.curvatureRadPerMeter = getCurvature();
        }
      }
      return new Trajectory(states);
    }

    /**
     * Calculates the curvature of the circular path
     * 
     * @return Curvature in radians per unit along the circumference
     */
    public double getCurvature() {
      return (1 / radius) * (clockwise ? -1 : 1) * (reversed ? -1 : 1);
    }

    /**
     * Checks if the provided position falls within the circular path
     */
    public boolean contains(Pose2d testPosition) {
      if (Math.abs(testPosition.getTranslation().getDistance(center) - radius) <= 0.0005) {
        Translation2d centerToCurrent = testPosition.getTranslation().minus(center);
        Rotation2d rotationFromCenter = new Rotation2d(Math.atan2(centerToCurrent.getY(), centerToCurrent.getX()));

        Rotation2d expectedRotation = rotationFromCenter
            .plus(Rotation2d.fromDegrees((clockwise ? -90 : 90) * (reversed ? -1 : 1)));
        if (Math.abs(testPosition.getRotation().minus(expectedRotation).getDegrees()) < 0.02) {
          double relativeCurrentDegrees = rotationFromCenter.minus(startingRotation).getDegrees();
          double relativeEndingDegrees = endingRotation.minus(startingRotation).getDegrees();
          if (clockwise) {
            if (relativeCurrentDegrees > 0) {
              relativeCurrentDegrees -= 360;
            }
            if (relativeEndingDegrees > 0) {
              relativeEndingDegrees -= 360;
            }
            if (relativeCurrentDegrees > relativeEndingDegrees) {
              return true;
            }
          } else {
            if (relativeCurrentDegrees < 0) {
              relativeCurrentDegrees += 360;
            }
            if (relativeEndingDegrees < 0) {
              relativeEndingDegrees += 360;
            }
            if (relativeCurrentDegrees < relativeEndingDegrees) {
              return true;
            }
          }
        }
      }
      return false;
    }
  }

  /**
   * Limits velocity and acceleration based on drive kinematics and other
   * constraints for a circle path. The purpose of this constraint is to pass
   * through data (with corrected curvature) to standard constraints.
   */
  private static class CirclePathConstraint implements TrajectoryConstraint {
    private final CirclePath circlePath;
    private final double calculatedMaxVelocity; // The calculated maximum velocity for the robot which doesn't exceed
                                                // the maximum velocity for the outer wheel
    private final List<TrajectoryConstraint> constraints;

    public CirclePathConstraint(CirclePath circlePath, DifferentialDriveKinematics driveKinematics, double maxVelocity,
        List<TrajectoryConstraint> constraints) {
      this.circlePath = circlePath;
      this.constraints = constraints;

      // Calculate maximum velocity using drive kinematics
      Rotation2d circleLengthRotation;
      if (circlePath.clockwise) {
        circleLengthRotation = circlePath.startingRotation.minus(circlePath.endingRotation);
      } else {
        circleLengthRotation = circlePath.endingRotation.minus(circlePath.startingRotation);
      }

      double circleLengthDegrees = circleLengthRotation.getDegrees();
      if (circleLengthDegrees < 0) {
        circleLengthDegrees += 360;
      }

      double circleLengthMetersOuter = (circleLengthDegrees / 360)
          * ((circlePath.radius + (driveKinematics.trackWidthMeters / 2)) * 2 * Math.PI);
      double maxDuration = circleLengthMetersOuter / maxVelocity;

      double circleLengthMetersCenter = (circleLengthDegrees / 360) * (circlePath.radius * 2 * Math.PI);
      calculatedMaxVelocity = circleLengthMetersCenter / maxDuration;
    }

    @Override
    public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
        double velocityMetersPerSecond) {
      if (circlePath.contains(poseMeters)) {
        double result = calculatedMaxVelocity;
        for (var i = 0; i < constraints.size(); i++) {
          double constraintResult = constraints.get(i).getMaxVelocityMetersPerSecond(poseMeters,
              circlePath.getCurvature(), velocityMetersPerSecond);
          result = constraintResult < result ? constraintResult : result;
        }
        return result;
      }
      return Double.MAX_VALUE;
    }

    @Override
    public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
        double velocityMetersPerSecond) {
      if (circlePath.contains(poseMeters)) {
        double minResult = -Double.MAX_VALUE;
        double maxResult = Double.MAX_VALUE;
        for (var i = 0; i < constraints.size(); i++) {
          MinMax constraintResult = constraints.get(i).getMinMaxAccelerationMetersPerSecondSq(poseMeters,
              circlePath.getCurvature(), velocityMetersPerSecond);
          minResult = constraintResult.minAccelerationMetersPerSecondSq > minResult
              ? constraintResult.minAccelerationMetersPerSecondSq
              : minResult;
          maxResult = constraintResult.maxAccelerationMetersPerSecondSq < maxResult
              ? constraintResult.maxAccelerationMetersPerSecondSq
              : maxResult;
        }
        return new MinMax(minResult, maxResult);
      }
      return new MinMax(-Double.MAX_VALUE, Double.MAX_VALUE);
    }
  }
}
