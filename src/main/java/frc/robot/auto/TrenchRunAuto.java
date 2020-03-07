/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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
import java.util.List;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.auto.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrenchRunAuto extends SequentialCommandGroup {
  /**
   * Starts with a step back shot.  Then picks up the 5 balls in the trench and shoots them.
   */
  static Drive mDrive = Drive.getInstance();

  public TrenchRunAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new StepBackAndShootAuto(),
      new ParallelCommandGroup(
        new RevFlywheel(2300),
        driveTrajectory(trenchRun()),
        new StartIntakeCommand()
        //also turn on the intake and start grabbing balls
      ),
      new StopIntakeCommand(),
      driveTrajectory(upTheTrench()),
      new ShootCommand().withTimeout(3)
    );
  }

  private static Trajectory upTheTrench(){
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(mDrive.getFeedforward(),
        mDrive.getKinematics(),
        10);

    var turnConstraint = new CentripetalAccelerationConstraint(2);
        
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Constants.kMaxSpeedMetersPerSecond),
    Units.feetToMeters(Constants.kMaxAccelerationMetersPerSecondSquared));
    config.setKinematics(mDrive.getKinematics());
    config.addConstraint(autoVoltageConstraint);
    config.setReversed(false);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(-3, -1.5, new Rotation2d(0)),
        new Pose2d(0, -1.5, new Rotation2d(0))
      ),
      // Pass config
      config
    );

    return trajectory;
  }

  private static Trajectory trenchRun(){
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
        new Pose2d(-1, 0, new Rotation2d(0)),
        new Pose2d(-3, -1.5, new Rotation2d(0))
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
    return "Trench Run Auto";
  }
}
