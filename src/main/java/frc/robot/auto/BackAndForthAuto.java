/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import frc.robot.auto.AutoTrajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BackAndForthAuto extends SequentialCommandGroup {
  /**
   * A test autonomous to see how to run more than one trajectory in one command
   */
  static Drive mDrive = Drive.getInstance();
  public BackAndForthAuto() {
    
    super(
      driveTrajectory(new AutoTrajectories().aMeterBack()),
      driveTrajectory(new AutoTrajectories().aMeterForward())
    );
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
    return "Back and Forth Auto";
  }
}
