package frc.robot.commands;

import frc.robot.subsystems.CameraSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Sets the current camera
 */
public class SetCamera extends InstantCommand {

  private boolean frontCamera;
  private final CameraSystem cameraSubsystem;

  public SetCamera(CameraSystem cameraSubsystem, boolean useFrontCamera) {
    this.cameraSubsystem = cameraSubsystem;
    frontCamera = useFrontCamera;
    addRequirements(cameraSubsystem);
  }

  // Called once when the command executes
  public void initialize() {
    if (frontCamera) {
      cameraSubsystem.useFrontCamera();
    } else {
      cameraSubsystem.useSecondCamera();
    }
    SmartDashboard.putString("Current Camera", frontCamera ? "Front" : "Second");
  }

}
