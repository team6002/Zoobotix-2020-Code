/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


public class driveForward extends CommandBase {
  /**
   * Command to drive forward a distance based on the trapezoid profile in drive
   */
  Drive mDrive = Drive.getInstance();
  int goal;
  public driveForward(int desired) {
    
    goal = desired;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.setGoal(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //updating profile is taken care of in drive loop.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.finishedGoal();
  }
}
