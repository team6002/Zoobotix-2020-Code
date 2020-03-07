/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

public class RevFlywheel extends CommandBase {
  /**
   * Starts to rev up the flywheel to a given rpm.
   * Also starts targeting
   */
  Shooter mShooter;
  Superstructure mSuperstructure;
  double setpoint = 0;
  public RevFlywheel(double speed) {
    mSuperstructure = Superstructure.getInstance();
    mShooter = Shooter.getInstance();
    setpoint = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSuperstructure.setWantedState(WantedState.SHOOT);
    mShooter.setVelocity(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(interrupted){
    //   mShooter.setOpenLoop(0);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mShooter.getIsReady();
  }
}
