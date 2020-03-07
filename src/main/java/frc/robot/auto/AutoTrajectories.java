/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.geometry.*;
import java.util.List;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * A file to store all the trajectories used in autonomous
 */
public class AutoTrajectories {
    static Drive mDrive;

    public AutoTrajectories() {
        mDrive = Drive.getInstance();
    }

    public Trajectory aMeterBack(){
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

    public Trajectory aMeterForward(){
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
          new Pose2d(-1, 0, new Rotation2d(0)),
          new Pose2d(0, 0, new Rotation2d(0))
        ),
        // Pass config
        config
      );
  
      return trajectory;
    }

    public Trajectory OppositeTrenchToShootingPosition() {
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(mDrive.getFeedforward(),
              mDrive.getKinematics(), 10);

      var turnConstraint = new CentripetalAccelerationConstraint(2);

      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Constants.kMaxSpeedMetersPerSecond),
              Units.feetToMeters(Constants.kMaxAccelerationMetersPerSecondSquared));
      config.setKinematics(mDrive.getKinematics());
      config.addConstraint(autoVoltageConstraint);
      config.setReversed(false);
  
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(1.3, 0.08),
          new Translation2d(2.77, -1.12)
        ),
        new Pose2d(3, -1.4, new Rotation2d(-0.15)),
        // Pass config
        config
      );
  
      return trajectory;
    }

    public Trajectory RendevouzToShooting(){//needs tuning
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(mDrive.getFeedforward(),
        mDrive.getKinematics(), 10);

      var turnConstraint = new CentripetalAccelerationConstraint(2);

      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(Constants.kMaxSpeedMetersPerSecond),
              Units.feetToMeters(Constants.kMaxAccelerationMetersPerSecondSquared));
      config.setKinematics(mDrive.getKinematics());
      config.addConstraint(autoVoltageConstraint);
      config.setReversed(false);
  
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(1.3, 0.08),
          new Translation2d(2.77, -1.12)
        ),
        new Pose2d(3, -1.4, new Rotation2d(-0.15)),
        // Pass config
        config
      );
  
      return trajectory;
    }
}
