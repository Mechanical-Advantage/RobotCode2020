/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean tuningMode = false;
    public static final double loopPeriodSeconds = 0.02;
    public static final RobotType defaultRobot = RobotType.ROBOT_2020;
    private static RobotType robot;

    public static final double fieldLength = 52 * 12 + 5.25;
    public static final double fieldWidth = 26 * 12 + 11.25;
    public static final double initiationLine = 120; // From player station 2
    public static final double visionTargetHorizDist = 43.75 + 24; // From center of field
    // Trench run is against both edges of the field side to side
    public static final double trenchRunWidth = (4 * 12) + 7.5;
    // Trench run is centered end to end on the field
    public static final double trenchRunLength = 18 * 12;

    public static RobotType getRobot() {
        return robot;
    }

    public static void setRobot(RobotType robot) {
        if (Constants.robot == null) {
            Constants.robot = robot;
        }
    }

    public enum RobotType {
        ROBOT_2020, ROBOT_2020_DRIVE, ROBOT_2019, ORIGINAL_ROBOT_2018, NOTBOT, REBOT
    }
}
