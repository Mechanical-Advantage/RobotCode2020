/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

/**
 * Manages a school zone
 */
public class SchoolZone {

  private double limit;
  private double normalPercent;
  private double lowerStart;
  private double upperStart;
  private boolean previouslyLimitedForward;
  private boolean previouslyLimitedReverse;
  private BaseMotorController[] motorControllers;

  /**
   * Creates a new SchoolZone. If motor controllers are provided, they will have
   * their peak limits updated as positions are applied (only if needed).
   */
  public SchoolZone(double limitPercent, double lowerStart, double upperStart,
      BaseMotorController... motorControllers) {
    this(limitPercent, 1, lowerStart, upperStart, motorControllers);
  }

  /**
   * Creates a new SchoolZone. If motor controllers are provided, they will have
   * their peak limits updated as positions are applied (only if needed).
   */
  public SchoolZone(double limitPercent, double normalPercent, double lowerStart, double upperStart,
      BaseMotorController... motorControllers) {
    limit = limitPercent;
    this.normalPercent = normalPercent;
    this.lowerStart = lowerStart;
    this.upperStart = upperStart;
    this.motorControllers = motorControllers;
  }

  /**
   * Apply the speed limit to motor controllers if needed.
   */
  private void applyToControllers(RequiredSpeedLimitChanges neededChanges, boolean applyForwardLimit,
      boolean applyReverseLimit) {
    if (neededChanges.includesForwardChange() && applyForwardLimit) {
      for (BaseMotorController controller : motorControllers) {
        controller.configPeakOutputForward(getForwardSpeedLimit());
      }
    }
    if (neededChanges.includesReverseChange() && applyReverseLimit) {
      for (BaseMotorController controller : motorControllers) {
        controller.configPeakOutputReverse(getReverseSpeedLimit());
      }
    }
  }

  /**
   * Set the limits on the motor controllers to the current speed limits without
   * checking previous state. This ensures that the limits match what is expected.
   */
  public void setControllerLimits() {
    for (BaseMotorController controller : motorControllers) {
      controller.configPeakOutputForward(getForwardSpeedLimit());
      controller.configPeakOutputReverse(getReverseSpeedLimit());
    }
  }

  /**
   * Evaluate the given position to determine if it is in a school zone
   * 
   * @param position The position to evaluate
   * @return Which speed limits need to be or were changed
   */
  public RequiredSpeedLimitChanges applyPosition(double position) {
    return applyPosition(position, true, true);
  }

  /**
   * Evaluate the given position to determine if it is in a school zone
   * 
   * @param position          The position to evaluate
   * @param applyForwardLimit Whether to apply the forward limit to controllers
   * @param applyReverseLimit Whether to apply the reverse limit to controllers
   * @return Which speed limits need to be or were changed
   */
  public RequiredSpeedLimitChanges applyPosition(double position, boolean applyForwardLimit,
      boolean applyReverseLimit) {
    boolean outsideForwardLimit = false;
    boolean outsideReverseLimit = false;
    RequiredSpeedLimitChanges neededChanges;
    if (position > upperStart) {
      outsideForwardLimit = true;
    }
    if (position < lowerStart) {
      outsideReverseLimit = true;
    }
    if (outsideForwardLimit != previouslyLimitedForward && outsideReverseLimit != previouslyLimitedReverse) {
      previouslyLimitedForward = outsideForwardLimit;
      previouslyLimitedReverse = outsideReverseLimit;
      neededChanges = RequiredSpeedLimitChanges.BOTH;
    } else if (outsideForwardLimit != previouslyLimitedForward) {
      previouslyLimitedForward = outsideForwardLimit;
      neededChanges = RequiredSpeedLimitChanges.FORWARD;
    } else if (outsideReverseLimit != previouslyLimitedReverse) {
      previouslyLimitedReverse = outsideReverseLimit;
      neededChanges = RequiredSpeedLimitChanges.REVERSE;
    } else {
      neededChanges = RequiredSpeedLimitChanges.NONE;
    }
    if (motorControllers.length > 0) {
      applyToControllers(neededChanges, applyForwardLimit, applyReverseLimit);
    }
    return neededChanges;
  }

  /**
   * Get the speed limit that should be in effect based on the last applied
   * position. Will be normal output (or 100%) if not in school zone.
   */
  public double getForwardSpeedLimit() {
    return previouslyLimitedForward ? limit : normalPercent;
  }

  /**
   * Get the speed limit that should be in effect based on the last applied
   * position. Will be normal output (or -100%) if not in school zone and will
   * always be negative.
   */
  public double getReverseSpeedLimit() {
    return -1 * (previouslyLimitedReverse ? limit : normalPercent);
  }

  public enum RequiredSpeedLimitChanges {
    NONE, FORWARD, REVERSE, BOTH;

    public boolean includesForwardChange() {
      switch (this) {
      case FORWARD:
      case BOTH:
        return true;
      default:
        return false;
      }
    }

    public boolean includesReverseChange() {
      switch (this) {
      case REVERSE:
      case BOTH:
        return true;
      default:
        return false;
      }
    }
  }
}
