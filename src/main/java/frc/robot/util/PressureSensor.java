/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class PressureSensor {

    private static final double minVoltage = 0.0; // voltage when no pressure
    private static final double voltageMultiplier = 1.0; // multiplier to convert voltage to pressure
    private AnalogInput sensor;

    public PressureSensor(int channel) {
        sensor = new AnalogInput(channel);
        sensor.setAverageBits(4);
    }

    public double getPressure() {
        return (sensor.getAverageVoltage() - minVoltage) * voltageMultiplier;
    }

    public double getVoltage() {
        return sensor.getAverageVoltage();
    }
}
