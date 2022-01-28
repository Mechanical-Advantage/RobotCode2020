/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.oi.IOperatorOI.SetPressureInterface;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

/**
 * Subsystem for managing the compressor and pressure sensor.
 */
public class Pneumatics extends SubsystemBase {

  private static final double supplyNormalized = 4.5868055556;
  private static final double disconnectedVoltage = 0.35; // When output less than this value, assume that the pressure
                                                          // sensor is disconnected
  private AnalogInput sensor;
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private SetPressureInterface setPressure;

  private Timer noPressureTimer = new Timer();
  private Timer compressorEnabledTimer = new Timer();

  private Alert missingAlert = new Alert("Pressure sensor not connected, hood movement may be unpredictable.",
      AlertType.WARNING);
  private Alert dumpValveAlert = new Alert("Cannot build pressure. Is the dump value open?", AlertType.WARNING);

  public Pneumatics(int sensorChannel, SetPressureInterface setPressure) {
    this.setPressure = setPressure;
    if (available()) {
      sensor = new AnalogInput(sensorChannel);
      sensor.setAverageBits(4);
    }
    noPressureTimer.reset();
    noPressureTimer.start();
    compressorEnabledTimer.reset();
    compressorEnabledTimer.start();
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

  public double getSensorVoltage() {
    if (available()) {
      return sensor.getAverageVoltage();
    } else {
      return 0;
    }
  }

  public boolean isSensorConnected() {
    return getSensorVoltage() >= disconnectedVoltage;
  }

  @Override
  public void periodic() {
    if (available()) {
      SmartDashboard.putNumber("Pressure Sensor", getPressure());
      setPressure.set(getPressure());
      missingAlert.set(getSensorVoltage() < disconnectedVoltage);

      // Detect if dump value is open
      if (getPressure() > 1) {
        noPressureTimer.reset();
      }
      if (!compressor.enabled()) {
        compressorEnabledTimer.reset();
      }
      dumpValveAlert.set(isSensorConnected() && noPressureTimer.hasElapsed(5) && compressorEnabledTimer.hasElapsed(5));
    } else {
      missingAlert.set(false);
      dumpValveAlert.set(false);
    }
  }
}
