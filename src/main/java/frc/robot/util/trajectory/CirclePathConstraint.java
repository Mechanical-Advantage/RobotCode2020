// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

/**
 * Limits velocity and acceleration using a different constraint for a circle
 * path. The purpose of this constraint is to pass through data (with corrected
 * curvature) to a standard constraint.
 */
public class CirclePathConstraint implements TrajectoryConstraint {
  private final CirclePath circlePath;
  private final TrajectoryConstraint baseConstraint;

  public CirclePathConstraint(CirclePath circlePath, TrajectoryConstraint baseConstraint) {
    this.circlePath = circlePath;
    this.baseConstraint = baseConstraint;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    if (circlePath.contains(poseMeters)) {
      return baseConstraint.getMaxVelocityMetersPerSecond(poseMeters, circlePath.getCurvature(),
          velocityMetersPerSecond);
    }
    return Double.MAX_VALUE;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    if (circlePath.contains(poseMeters)) {
      return baseConstraint.getMinMaxAccelerationMetersPerSecondSq(poseMeters, circlePath.getCurvature(),
          velocityMetersPerSecond);
    }
    return new MinMax(-Double.MAX_VALUE, Double.MAX_VALUE);
  }
}