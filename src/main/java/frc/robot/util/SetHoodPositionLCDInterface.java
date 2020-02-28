/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.subsystems.ShooterHood.HoodPosition;

/**
 * A functional interface for the setHoodPosition() method of operatorOI
 */
@FunctionalInterface
public interface SetHoodPositionLCDInterface {
    void set(HoodPosition position);
}
