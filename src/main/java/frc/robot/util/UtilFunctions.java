/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Contains simple utility functions.
 */
public class UtilFunctions {
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Converts the input to -179 to 180
     * 
     * @param input The input angle
     * @return The resulting angle
     */
    public static double boundHalfDegrees(double input) {
        while (input > 180) {
            input -= 360;
        }
        while (input <= -180) {
            input += 360;
        }
        return input;
    }
}
