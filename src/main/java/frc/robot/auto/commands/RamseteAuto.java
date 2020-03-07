/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Holds onto the ramsete controller for us so we don't have to copy and paste
 * it everywhere.
 */
public class RamseteAuto {
    static Drive mDrive;

    public RamseteAuto() {
        mDrive = Drive.getInstance();
    }

    public static Command driveTrajectory(Trajectory trajectory) {

        trajectory.relativeTo(mDrive.getPose());
    
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
}
