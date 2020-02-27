/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.util.TunableNumber;

public class RunClimber extends CommandBase {

  private TunableNumber setpoint = new TunableNumber("Climber/setpoint");
  private final Climber climber;
  private DoubleSupplier oiClimberY;
  private DoubleSupplier oiGetDeadband;

  /**
   * Creates a new RunClimber.
   *
   * @param Climber
   */
  public RunClimber(Climber climberSub, DoubleSupplier climberY, DoubleSupplier getDeadband) {
    climber = climberSub;
    addRequirements(climberSub);
    oiClimberY = climberY;
    oiGetDeadband = getDeadband;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint.setDefault(0);
    climber.deploy();
    climber.run(setpoint.get());
  }

  private double processJoystickAxis(double joystickAxis) {
    // cube to improve low speed control, multiply by -1 because negative joystick
    // means forward, 0 if within deadband
    return Math.abs(joystickAxis) > oiGetDeadband.getAsDouble() ? joystickAxis * Math.abs(joystickAxis) * -1 : 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.tuningMode) {
      climber.run(setpoint.get());
    }

    double joystick = 0;
    joystick = processJoystickAxis(oiClimberY.getAsDouble());
    climber.run(joystick);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
