/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Since we don't have robot container, this will hold the autonomous command to
 * be run during autonomous.
 */
public class AutoContainer {
    private Drive mDrive = Drive.getInstance();

    public Command getAutonomousCommand(){
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                mDrive.getFeedforward(),
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

        trajectory.relativeTo(mDrive.getPose());


        SmartDashboard.putNumber("Time of trajectory", trajectory.getTotalTimeSeconds());

        RamseteController disabledRamsete = new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        };
        PIDController leftController = new PIDController(Constants.kDrivePVel,0,0);
        PIDController rightController = new PIDController(Constants.kDrivePVel,0,0);
        
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            mDrive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            //disabledRamsete,
            mDrive.getFeedforward(),
            mDrive.getKinematics(),
            mDrive::getWheelSpeeds,
            leftController, 
            rightController,
            // RamseteCommand passes volts to the callback
            (leftVolts, rightVolts) -> {
                mDrive.tankDriveVolts(leftVolts, rightVolts);
        
                SmartDashboard.putNumber("left Wanted Speed", leftController.getSetpoint());
                SmartDashboard.putNumber("right Wanted Speed", rightController.getSetpoint());
            },
            //mDrive::tankDriveVolts,
            mDrive
        );
        
        return ramseteCommand.andThen(() -> mDrive.tankDriveVolts(0, 0));
    }

}
