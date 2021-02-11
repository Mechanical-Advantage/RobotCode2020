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
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.tools.pathview.TrajectoryVisualizer;

public class NewRunMotionProfile extends CommandBase {

  private static final double kRamseteB = 0.0025; // 0.05 seems to be equivalent to the recommendation for meters
  private static final double kRamseteZeta = 0.7;
  private static final double maxVoltage = 10; // WPILib docs suggest less than 12 because of voltage drop

  private double kS; // Volts
  private double kV; // Volt seconds per inch
  private double kA; // Volt seconds squared per inch
  private double trackWidth;
  private double maxVelocity; // in/s
  private double maxAcceleration; // in/s^2
  private double maxCentripetalAcceleration; // in/s^2

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
   * quintic spline
   * 
   * @param driveTrain      The drive train
   * @param odometry        The robot odometry
   * @param initialVelocity The velocity at the beginning of the profile
   * @param waypointData    The poses & circle paths including initial position
   * @param endVelocity     The target velocity at the end of the profile
   * @param reversed        Whether the robot drives backwards during theprofile
   * @param relative        Whether the profile is a relative change to the robot
   *                        position as opposed to field coordinates
   */

  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, double initialVelocity,
      List<Object> waypointData, double endVelocity, boolean reversed, boolean relative) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData);
    Pose2d initialPosition = this.waypointPoses.remove(0);
    useQuintic = true;

    // Identical to constructor for fixed position & cubic
    setup(driveTrain, odometry, null, null, endVelocity, reversed, false);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(initialPosition, initialVelocity);
  }

  /**
   * Creates a new RunMotionProfile that starts from a fixed position, using a
   * cubic spline
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
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d initialPosition,
      double initialVelocity, List<Translation2d> intermediatePoints, Pose2d endPosition, double endVelocity,
      boolean reversed, boolean relative) {
    // The setup function's relative trajectory handling is unneccessary with a
    // defined start point so always pass false and do the other neccessary logic
    // here
    updateConstants();
    setup(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, false);
    dynamicTrajectory = false;
    relativeTrajectory = relative;
    startGeneration(initialPosition, initialVelocity);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a quintic spline
   * 
   * @param driveTrain   The drive train
   * @param odometry     The robot odometry
   * @param waypointData The poses & circle paths after initial position
   * @param endVelocity  The target velocity at the end of the profile
   * @param reversed     Whether the robot drives backwards during theprofile
   * @param relative     Whether the profile is a relative change to the robot
   *                     position as opposed to field coordinates
   */
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Object> waypointData,
      double endVelocity, boolean reversed, boolean relative) {
    updateConstants();
    this.waypointPoses = processWaypointData(waypointData);
    useQuintic = true;
    setup(driveTrain, odometry, null, null, endVelocity, reversed, relative);
  }

  /**
   * Creates a new RunMotionProfile that starts from the robot's current position,
   * using a cubic spline
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
  public NewRunMotionProfile(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative) {
    updateConstants();
    setup(driveTrain, odometry, intermediatePoints, endPosition, endVelocity, reversed, relative);
  }

  /**
   * Updates constants based on current robot
   */
  @SuppressWarnings("incomplete-switch")
  private void updateConstants() {
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
        kS = .124;
        kV = .0722;
        kA = .00475;
        trackWidth = 25.934;
        maxVelocity = 130;
        maxAcceleration = 130;
        maxCentripetalAcceleration = 120;
        break;
    }
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
   */
  private void setup(DriveTrainBase driveTrain, RobotOdometry odometry, List<Translation2d> intermediatePoints,
      Pose2d endPosition, double endVelocity, boolean reversed, boolean relative) {
    this.driveTrain = driveTrain;
    if (driveTrain != null) {
      addRequirements(driveTrain);
    }
    this.odometry = odometry;
    dynamicTrajectory = true;
    driveKinematics = new DifferentialDriveKinematics(trackWidth);
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kS, kV, kA), driveKinematics, maxVoltage);
    CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(
        maxCentripetalAcceleration);
    this.intermediatePointsTranslations = intermediatePoints;
    this.endPosition = endPosition;
    config = new TrajectoryConfig(maxVelocity, maxAcceleration).setKinematics(driveKinematics)
        .addConstraint(voltageConstraint).addConstraint(centripetalAccelerationConstraint).setEndVelocity(endVelocity)
        .setReversed(reversed);
    for (int i = 0; i < circlePaths.size(); i++) {
      config.addConstraint(circlePaths.get(i).getConstraint(driveKinematics, maxVelocity,
          List.of(voltageConstraint, centripetalAccelerationConstraint)));
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
      trajectory = generator.getTrajectory();
      if (trajectory != null) {
        trajectory = adjustCircleTrajectories(trajectory);
      }
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
    if (followerCommand == null) {
      followerCommand = new RamseteCommand(trajectory, odometry::getCurrentPose,
          new RamseteController(kRamseteB, kRamseteZeta), driveKinematics, driveTrain::driveInchesPerSec);
    }
    followerCommand.schedule();
    followerStarted = true;
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Runs the trajectory visualizer on this command. This should not be called
   * from robot code, but instead used only when testing on development machines.
   * It can be used by putting a main method in the command file of the profile,
   * and then running that main method from within VSCode - this ensures it is
   * never called on the robot on accident.
   *
   * This version of the method should be used for profiles that have a starting
   * position which is determined by the current position of the robot. Since
   * there is no real robot in the visualization, a fake initial robot position
   * must be provided
   * 
   * @param ppi                  The number of pixels which should represent one
   *                             inch. 2.5 is a good starting value
   * @param markers              A list of positions to draw "markers" (7 inch
   *                             magenta circles) on.
   * @param initialRobotPosition The starting position of the robot to test with
   */
  public void visualize(double ppi, List<Translation2d> markers, Pose2d initialRobotPosition) {
    if (initialRobotPosition != null) {
      startGeneration(initialRobotPosition, 0.0);
    }
    // Busy-wait for trajectory to finish generating
    Trajectory t = null;
    while (t == null) {
      try {
        Thread.sleep(10); // Blocking busy-wait
      } catch (InterruptedException e) {
        return; // We got interrupted (probably user hit stop), so just exit
      }
      t = generator.getTrajectory(); // Attempt to grab new path
    }
    t = adjustCircleTrajectories(t);
    TrajectoryVisualizer viz = new TrajectoryVisualizer(ppi, t, trackWidth, markers);
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
   * @param ppi     The number of pixels which should represent one inch. 2.5 is a
   *                good starting value
   * @param markers A list of positions to draw "markers" (7 inch magenta circles)
   *                on.
   */
  public void visualize(double ppi, List<Translation2d> markers) {
    visualize(ppi, markers, null);
  }

  /**
   * Processes a list of Pose2d and CirclePath objects into only Pose2d objects,
   * including saving circle path objects
   */
  public List<Pose2d> processWaypointData(List<Object> waypointData) {
    List<Pose2d> outputPoses = new ArrayList<>();
    for (int i = 0; i < waypointData.size(); i++) {
      if (waypointData.get(i).getClass() == Pose2d.class) {
        outputPoses.add((Pose2d) waypointData.get(i));
      } else if (waypointData.get(i).getClass() == CirclePath.class) {
        CirclePath circle = (CirclePath) waypointData.get(i);
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
    private static final double separationDistance = 0.1; // Distance between points (must be quite small to ensure
                                                          // continuous curve)

    public final Translation2d center;
    public final double radius;
    public final Rotation2d startingRotation;
    public final Rotation2d endingRotation;
    public final boolean clockwise;

    /**
     * Creates a circular path with the given properties
     * 
     * @param center           The center position of the circle
     * @param radius           The radius of the circle
     * @param startingRotation The rotation relative to the center at which to start
     *                         the path (NOT the starting rotation of the robot)
     * @param endingRotation   The rotation relative to the center at which to end
     *                         the path (NOT the ending rotation of the robot)
     * @param clockwise        Whether to move clockwise or countercloswise from the
     *                         start to end
     */
    public CirclePath(Translation2d center, double radius, Rotation2d startingRotation, Rotation2d endingRotation,
        boolean clockwise) {
      this.center = center;
      this.radius = radius;
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
            Rotation2d.fromDegrees(clockwise ? -90 : 90));
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
      return (1 / radius) * (clockwise ? -1 : 1);
    }

    /**
     * Checks if the provided position falls within the circular path
     */
    public boolean contains(Pose2d testPosition) {
      if (Math.abs(testPosition.getTranslation().getDistance(center) - radius) <= 0.02) {
        Translation2d centerToCurrent = testPosition.getTranslation().minus(center);
        Rotation2d rotationFromCenter = new Rotation2d(Math.atan2(centerToCurrent.getY(), centerToCurrent.getX()));

        Rotation2d expectedRotation = rotationFromCenter.plus(Rotation2d.fromDegrees(clockwise ? -90 : 90));
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

      double circleLengthInchesOuter = (circleLengthDegrees / 360)
          * ((circlePath.radius + (driveKinematics.trackWidthMeters / 2)) * 2 * Math.PI);
      double maxDuration = circleLengthInchesOuter / maxVelocity;

      double circleLengthInchesCenter = (circleLengthDegrees / 360) * (circlePath.radius * 2 * Math.PI);
      calculatedMaxVelocity = circleLengthInchesCenter / maxDuration;
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
