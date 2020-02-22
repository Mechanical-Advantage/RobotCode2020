/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ShooterHood extends SubsystemBase {

  private static final int liftSolenoidChannel = 6;
  private static final int stopSolenoidChannel = 5;

  private Solenoid liftSolenoid;
  private Solenoid stopSolenoid;

  /**
   * Creates a new ShooterHood.
   */
  public ShooterHood() {
    if (Constants.getRobot() != RobotType.ROBOT_2020 && Constants.getRobot() != RobotType.ROBOT_2020_DRIVE) {
      return;
    }
    liftSolenoid = new Solenoid(liftSolenoidChannel);
    stopSolenoid = new Solenoid(stopSolenoidChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the position of the lift solenoid
   * 
   * @param raised
   */
  public void setLiftPosition(boolean raised) {
    liftSolenoid.set(raised);
  }

  /**
   * Sets whether the stop solenoid is extended
   * 
   * @param stopped
   */
  public void setStopPosition(boolean stopped) {
    stopSolenoid.set(stopped);
  }
}
