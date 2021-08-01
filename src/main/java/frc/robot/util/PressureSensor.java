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
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.SetPressureInterface;

/**
 * Class for the REV analog pressure sensor.
 */
public class PressureSensor extends SubsystemBase {

    private static final double supplyNormalized = 4.5868055556;
    private AnalogInput sensor;
    private SetPressureInterface setPressure;

    public PressureSensor(int channel, SetPressureInterface setPressure) {
        this.setPressure = setPressure;
        if (available()) {
            sensor = new AnalogInput(channel);
            sensor.setAverageBits(4);
        }
    }

    public boolean available() {
        return Constants.getRobot() == RobotType.ROBOT_2020;
    }

    public double getPressure() {
        if (available()) {
            double pressure = ((sensor.getAverageVoltage() / supplyNormalized) * 250) - 25;
            return pressure < 0 ? 0 : pressure;
        } else {
            return 0;
        }
    }

    public double getVoltage() {
        if (available()) {
            return sensor.getAverageVoltage();
        } else {
            return 0;
        }
    }

    @Override
    public void periodic() {
        if (available()) {
            SmartDashboard.putNumber("Pressure Sensor", getPressure());
            setPressure.set(getPressure());
        }
    }
}
