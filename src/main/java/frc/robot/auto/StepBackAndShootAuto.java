/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.auto.commands.*;


public class StepBackAndShootAuto extends SequentialCommandGroup {
  /**
   * Our simplest autonomous mode/
   * Starts moving back ~1 meter while reving the shooter.  Then starts shooting.
   */
  private static Drive mDrive = Drive.getInstance();

  public StepBackAndShootAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new RevFlywheel(2300), 
        driveTrajectory(meterOffLine())
      ), 
    new ShootCommand().withTimeout(3)
    );

  }

  private static Trajectory meterOffLine(){
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(mDrive.getFeedforward(),
        mDrive.getKinematics(),
        10);

    var turnConstraint = new CentripetalAccelerationConstraint(2);
        
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Constants.kMaxSpeedMetersPerSecond),
    Units.feetToMeters(Constants.kMaxAccelerationMetersPerSecondSquared));
    config.setKinematics(mDrive.getKinematics());
    config.addConstraint(autoVoltageConstraint);
    config.setReversed(true);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(0)),
        new Pose2d(-1, 0, new Rotation2d(0))
      ),
      // Pass config
      config
    );

    return trajectory;
  }

  public static Command driveTrajectory(Trajectory trajectory) {

    trajectory = trajectory.relativeTo(mDrive.getPose());

    PIDController leftController = new PIDController(Constants.kDrivePVel,0,0);
    PIDController rightController = new PIDController(Constants.kDrivePVel,0,0);

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        mDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        mDrive.getFeedforward(),
        mDrive.getKinematics(),
        mDrive::getWheelSpeeds,
        leftController, 
        rightController,
        // RamseteCommand passes volts to the callback
        mDrive::tankDriveVolts,
        mDrive
    );

    return ramseteCommand.andThen(() -> mDrive.tankDriveVolts(0, 0));
  }

  @Override
  public String toString(){
    return "Step Back And Shoot Auto";
  }

}
