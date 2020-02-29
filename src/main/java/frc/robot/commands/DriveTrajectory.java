/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveTrajectory extends CommandBase {
  /**
   * Creates a new DriveTrajectory.
   */
  Drive mDrive;
  public DriveTrajectory() {
    mDrive = Drive.getInstance();
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
}
