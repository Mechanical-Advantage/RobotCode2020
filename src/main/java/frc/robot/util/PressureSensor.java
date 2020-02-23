/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class for the REV analog pressure sensor.
 */
public class PressureSensor extends SubsystemBase {

    private static final double supplyNormalized = 4.9705882353;
    private AnalogInput sensor;

    public PressureSensor(int channel) {
        sensor = new AnalogInput(channel);
        sensor.setAverageBits(4);
    }

    public double getPressure() {
        return ((sensor.getAverageVoltage() / supplyNormalized) * 250) - 25;
    }

    public double getVoltage() {
        return sensor.getAverageVoltage();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pressure Sensor", getPressure());
    }
}
