// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import frckit.util.GeomUtil;

/**
 * Wraps a constraint defined in inches to convert inputs from meters to inches
 * and outputs from inches to meters.
 */
public class InchesToMetersConstraint implements TrajectoryConstraint {
  private final TrajectoryConstraint inchesConstraint;

  public InchesToMetersConstraint(TrajectoryConstraint inchesConstraint) {
    this.inchesConstraint = inchesConstraint;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    return Units.inchesToMeters(inchesConstraint.getMaxVelocityMetersPerSecond(GeomUtil.metersToInches(poseMeters),
        Units.metersToInches(curvatureRadPerMeter), Units.metersToInches(velocityMetersPerSecond)));
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    MinMax accelerationInches = inchesConstraint.getMinMaxAccelerationMetersPerSecondSq(
        GeomUtil.metersToInches(poseMeters), Units.metersToInches(curvatureRadPerMeter),
        Units.metersToInches(velocityMetersPerSecond));
    return new MinMax(Units.inchesToMeters(accelerationInches.minAccelerationMetersPerSecondSq),
        Units.inchesToMeters(accelerationInches.maxAccelerationMetersPerSecondSq));
  }
}