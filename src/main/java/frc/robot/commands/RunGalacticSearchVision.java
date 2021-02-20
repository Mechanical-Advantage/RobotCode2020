// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.GalacticSearchPipeline;

public class RunGalacticSearchVision extends CommandBase {
  private final VideoCapture video = new VideoCapture(0);
  private final GalacticSearchPipeline pipeline = new GalacticSearchPipeline();
  private Mat image;

  private GalacticSearchPath path;
  private Command runGalacticSearchABlue;
  private Command runGalacticSearchARed;
  private Command runGalacticSearchBBlue;
  private Command runGalacticSearchBRed;

  /** Creates a new RunGalacticSearchVision. */
  public RunGalacticSearchVision(RobotOdometry odometry, DriveTrainBase driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    runGalacticSearchABlue = new RunGalacticSearchABlue(odometry, driveTrain);
    runGalacticSearchARed = new RunGalacticSearchARed(odometry, driveTrain);
    runGalacticSearchBBlue = new RunGalacticSearchBBlue(odometry, driveTrain);
    runGalacticSearchBRed = new RunGalacticSearchBRed(odometry, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (path == null) {
      DriverStation.reportWarning("There is no path selected", false);
    } else {
      switch (path) {
        case A_BLUE:
          runGalacticSearchABlue.schedule();
          break;
        case A_RED:
          runGalacticSearchARed.schedule();
          break;
        case B_BLUE:
          runGalacticSearchBBlue.schedule();
          break;
        case B_RED:
          runGalacticSearchBRed.schedule();
          break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runGalacticSearchABlue.cancel();
    runGalacticSearchARed.cancel();
    runGalacticSearchBBlue.cancel();
    runGalacticSearchBRed.cancel();
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
      Mat hsvThreshold = pipeline.hsvThresholdOutput();

      // Check for balls
      if (checkForBall(hsvThreshold, 0.5, 0, 0, 3, 3)) {
        path = GalacticSearchPath.A_BLUE;
        SmartDashboard.putString("Path", "A BLUE");
      } else if (checkForBall(hsvThreshold, 0.5, 3, 3, 6, 6)) {
        path = GalacticSearchPath.A_RED;
        SmartDashboard.putString("Path", "A RED");
      } else if (checkForBall(hsvThreshold, 0.5, 6, 6, 9, 9)) {
        path = GalacticSearchPath.B_BLUE;
        SmartDashboard.putString("Path", "B BLUE");
      } else {
        path = GalacticSearchPath.B_RED;
        SmartDashboard.putString("Path", "B RED");
      }
    }
  }

  /**
   * Checks if a ball is in the specified area based on whether the # of white
   * pixels in an area is greater than a threshold. Origin in the upper left
   * 
   * @param source         The source image
   * @param whiteThreshold The % of the area that must be white
   * @param x1             Left column
   * @param y1             Top row
   * @param x2             Right column
   * @param y2             Bottom row
   */
  private boolean checkForBall(Mat source, double whiteThreshold, int x1, int y1, int x2, int y2) {
    Mat submat = source.submat(y1, y2, x1, x2);
    int whiteCount = 0;
    for (int x = 0; x < submat.width(); x++) {
      for (int y = 0; y < submat.height(); y++) {
        if (source.get(y, x)[0] > 0) {
          whiteCount++;

        }
      }
    }
    return whiteCount / (submat.width() * submat.height()) > whiteThreshold;
  }

  public static enum GalacticSearchPath {
    A_BLUE, A_RED, B_BLUE, B_RED
  }
}