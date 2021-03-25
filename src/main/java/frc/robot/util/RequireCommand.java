// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RequireCommand extends CommandBase {
  /**
   * Creates a new RequireCommand, which requires requires one or more subsystems
   * but does nothing. This is useful for temporarily blocking unwanted default
   * commands.
   */
  public RequireCommand(Subsystem... requirements) {
    addRequirements(requirements);
  }
}
