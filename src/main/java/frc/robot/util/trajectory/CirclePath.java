// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import frckit.util.GeomUtil;

/**
 * Represents a circular path to be used in a quintic spline
 **/
public class CirclePath {
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