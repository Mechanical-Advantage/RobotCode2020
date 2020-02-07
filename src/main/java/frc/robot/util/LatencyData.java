/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks a number of data point sand allows updating of old points which then
 * correct later points
 */
public class LatencyData implements DoubleSupplier {
  private double[] data;
  private double[] timestamps;
  private int currentIndex = 0;
  private int furthestIndex = 0; // Never resets, once curIndex has wrapped will always in length-1

  public LatencyData(int points) {
    data = new double[points];
    timestamps = new double[points];
  }

  public void addDataPoint(double point) {
    if (currentIndex > furthestIndex) {
      furthestIndex = currentIndex;
    }
    timestamps[currentIndex] = Timer.getFPGATimestamp(); // Java also has System.getNanoTime() which might work
    data[currentIndex] = point;
    currentIndex++;
    if (currentIndex >= data.length) {
      currentIndex = 0;
    }
  }

  public double getCurrentPoint() {
    return data[Math.floorMod(currentIndex - 1, data.length)];
  }

  /**
   * Correct all points since timestamp based on point
   * 
   * @returns Whether the point was successfully applied
   */
  public boolean addCorrectedData(double point, double timestamp) {
    PointAndIndexes originalPointAndIndexes = getPointAndIndexes(timestamp);
    if (originalPointAndIndexes == null) {
      return false;
    }
    double originalPoint = originalPointAndIndexes.getPoint();
    double difference = point - originalPoint;
    // Iternate from currentIndex-1 (most recent point) backwards to indexBefore,
    // apply difference
    for (int i = Math.floorMod((currentIndex - 1), data.length); Math.floorMod(i,
        data.length) != originalPointAndIndexes.getIndexBefore(); i--) {
      data[Math.floorMod(i, data.length)] += difference;
    }
    return true;
  }

  private PointAndIndexes getPointAndIndexes(double timestamp) {
    int indexBefore = Math.floorMod((currentIndex - 1), data.length);
    while (timestamps[indexBefore] > timestamp) {
      indexBefore--;
      indexBefore = Math.floorMod(indexBefore, data.length);
      if (timestamps[indexBefore] == 0 || indexBefore == currentIndex) {
        // No valid data is there for the time or the time is too far back
        return null;
      }
    }
    int indexAfter = (indexBefore + 1) % data.length;
    // Linear interpolation
    double point = data[indexBefore] + (timestamp - timestamps[indexBefore])
        * ((data[indexAfter] - data[indexBefore]) / (double) (timestamps[indexAfter] - timestamps[indexBefore]));
    return new PointAndIndexes(point, indexBefore, indexAfter);
  }

  public Double getPoint(double timestamp) {
    PointAndIndexes pointAndIndexes = getPointAndIndexes(timestamp);
    if (pointAndIndexes != null) {
      return pointAndIndexes.getPoint();
    } else {
      return null;
    }
  }

  public void clear() {
    for (int i = 0; i < data.length; i++) {
      data[i] = 0;
      timestamps[i] = 0;
    }
    currentIndex = 0;
    furthestIndex = 0;
  }

  @Override
  public double getAsDouble() {
    return getCurrentPoint();
  }

  private static class PointAndIndexes {
    private double point;
    private int indexBefore, indexAfter;

    public PointAndIndexes(double point, int indexBefore, int indexAfter) {
      this.point = point;
      this.indexBefore = indexBefore;
      this.indexAfter = indexAfter;
    }

    /**
     * @return the point
     */
    public double getPoint() {
      return point;
    }

    /**
     * @return the indexBefore
     */
    public int getIndexBefore() {
      return indexBefore;
    }

    /**
     * @return the indexAfter
     */
    @SuppressWarnings("unused")
    public int getIndexAfter() {
      return indexAfter;
    }
  }
}
