/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.RobotOdometry;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.drive.DriveTrainBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PointAtTargetAndShoot extends SequentialCommandGroup {

  /**
   * Creates a new PointAtTargetAndShoot.
   */
  public PointAtTargetAndShoot(DriveTrainBase driveTrain, RobotOdometry odometry, LimelightInterface limelight,
      AHRS ahrs, Hopper hopper, ShooterRoller roller, ShooterFlyWheel flywheel, ShooterHood hood, Intake intake,
      boolean driveForward) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
        new SequentialCommandGroup(
            new ParallelCommandGroup(new PointAtTargetWithOdometry(driveTrain, odometry, limelight).withTimeout(3),
                new WaitCommand(1).andThen(new WaitCommand(6).withInterrupt(() -> flywheel.atSetpoint()))),
            new WaitUntilCommand(() -> hood.atTargetPosition()),
            new ParallelRaceGroup(new RunHopper(hopper), new RunShooterRoller(roller), new HoldPosition(driveTrain),
                new RunIntakeForwards(intake), new WaitCommand(1.5)))
                    .deadlineWith(new RunShooterAtDistance(flywheel, hood, odometry, true)),
        new DriveDistanceOnHeading(driveTrain, ahrs, driveForward ? 60 : -60));
  }
}
