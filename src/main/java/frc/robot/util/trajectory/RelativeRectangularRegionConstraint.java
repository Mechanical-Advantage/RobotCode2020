// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

/**
 * Enforces a particular constraint only when a point enters a rectangular
 * region relative to the robot.
 */
public class RelativeRectangularRegionConstraint implements TrajectoryConstraint {
  private final Translation2d corner1;
  private final Translation2d corner2;
  private final TrajectoryConstraint constraint;
  private final Translation2d point;

  /**
   * Creates a new RelativeRectangularRegionConstraint. If you're using the same
   * region for multiple constraints, you can provide null as the point here and
   * use the forPoint decorator to create each new constraint.
   * 
   * @param corner1    Corner of rectangular region relative to the robot
   * @param corner2    Corner of rectangular region relative to the robot
   *                   (opposite of corner1)
   * @param point      The point on the field at which to enforce the constraint
   * @param constraint The constraint to enforce
   */
  public RelativeRectangularRegionConstraint(Translation2d corner1, Translation2d corner2,
      TrajectoryConstraint constraint, Translation2d point) {
    this.corner1 = corner1;
    this.corner2 = corner2;
    this.point = point;
    this.constraint = constraint;
  }

  /**
   * Returns a new RelativeRectangularRegionConstraint using the specified point.
   * The region and constraint are identical to the original version. This is
   * often useful when providing the same constraint for multiple points.
   */
  public RelativeRectangularRegionConstraint forPoint(Translation2d point) {
    return new RelativeRectangularRegionConstraint(corner1, corner2, constraint, point);
  }

  @Override
  public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    if (point != null) {
      if (pointInRegion(poseMeters)) {
        return constraint.getMaxVelocityMetersPerSecond(poseMeters, curvatureRadPerMeter, velocityMetersPerSecond);
      }
    }
    return Double.MAX_VALUE;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
      double velocityMetersPerSecond) {
    if (point != null) {
      if (pointInRegion(poseMeters)) {
        return constraint.getMinMaxAccelerationMetersPerSecondSq(poseMeters, curvatureRadPerMeter,
            velocityMetersPerSecond);
      }
    }
    return new MinMax(-Double.MAX_VALUE, Double.MAX_VALUE);
  }

  private boolean pointInRegion(Pose2d robotPose) {
    Translation2d relativePoint = new Pose2d(point, new Rotation2d()).relativeTo(robotPose).getTranslation();
    boolean xInRegion = (relativePoint.getX() < corner1.getX()) != (relativePoint.getX() < corner2.getX());
    boolean yInRegion = (relativePoint.getY() < corner1.getY()) != (relativePoint.getY() < corner2.getY());
    return xInRegion && yInRegion;
  }
}