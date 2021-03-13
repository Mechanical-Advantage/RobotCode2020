// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.drive.DriveTrainBase;
import frckit.util.GeomUtil;

public class DriveToPoint extends CommandBase {

  private final DriveTrainBase driveTrain;
  private final RobotOdometry odometry;
  private final Pose2d target;

  private final RamseteController controller = new RamseteController(2, 0.7);
  private final DifferentialDriveKinematics kinematics;
  private static final Pose2d tolerance = new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(1));

  /** Creates a new DriveToPoint. */
  public DriveToPoint(DriveTrainBase driveTrain, RobotOdometry odometry, Pose2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.odometry = odometry;
    this.target = target;
    addRequirements(driveTrain);
    controller.setTolerance(GeomUtil.inchesToMeters(tolerance));

    double trackWidth;
    switch (Constants.getRobot()) {
    case ROBOT_2019:
      trackWidth = 27.5932064868814;
      break;
    case ROBOT_2020_DRIVE:
      trackWidth = 24.890470780033485;
      break;
    case ROBOT_2020:
      trackWidth = 25.934;
      break;
    default:
      trackWidth = 1;
    }
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidth));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = controller.calculate(GeomUtil.inchesToMeters(odometry.getCurrentPose()),
        GeomUtil.inchesToMeters(target), 0, 0);
    DifferentialDriveWheelSpeeds differentialSpeeds = kinematics.toWheelSpeeds(speeds);
    driveTrain.driveInchesPerSec(Units.metersToInches(differentialSpeeds.leftMetersPerSecond),
        Units.metersToInches(differentialSpeeds.rightMetersPerSecond));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
