// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.GalacticSearchPipeline;

public class RunGalacticSearchVision extends CommandBase {
  private final VideoCapture video = new VideoCapture(0);
  private final GalacticSearchPipeline pipeline = new GalacticSearchPipeline();
  private Mat image;

  /** Creates a new RunGalacticSearchVision. */
  public RunGalacticSearchVision() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  public void updateVision() {
    // Capture an image from the camera
    if (video.read(image)) {

      // Run the GRIP pipeline
      pipeline.process(image);
      Mat output = pipeline.hsvThresholdOutput();
    }
  }
}
