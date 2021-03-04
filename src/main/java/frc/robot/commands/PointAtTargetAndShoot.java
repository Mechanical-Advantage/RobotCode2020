/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.oi.IOperatorOI.SetHoodPositionLCDInterface;
import frc.robot.oi.IOperatorOI.UpdateLEDInterface;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.ShooterFlyWheel;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.ShooterHood.HoodPosition;
import frc.robot.subsystems.drive.DriveTrainBase;
import frc.robot.util.PressureSensor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PointAtTargetAndShoot extends ParallelDeadlineGroup {

  /**
   * Creates a new PointAtTargetAndShoot.
   */
  public PointAtTargetAndShoot(DriveTrainBase driveTrain, LimelightInterface limelight, AHRS ahrs, Hopper hopper,
      ShooterRoller roller, ShooterFlyWheel flywheel, ShooterHood hood, PressureSensor pressureSensor,
      UpdateLEDInterface updateLED, SetHoodPositionLCDInterface setHoodLCD) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new PointAtTarget(driveTrain, limelight, ahrs)
        .alongWith(new InstantCommand(() -> hood.setTargetPosition(HoodPosition.LINE)))
        .alongWith(new WaitCommand(7).withInterrupt(() -> flywheel.getSpeed() > 6000)).andThen(new WaitCommand(0.5))
        .andThen(new RunHopper(hopper).alongWith(new RunShooterRoller(roller)).withTimeout(5))
        .andThen(new DriveDistanceOnHeading(driveTrain, ahrs, -60)), new RunShooterFlyWheel(flywheel));
  }
}
