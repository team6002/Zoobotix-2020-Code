/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Superstructure.WantedState;

public class ShootCommand extends CommandBase {
  /**
   * This program is dedicated to shooting the balls in auto
   */
  Shooter mShooter;
  Intake mIntake;
  Superstructure mSuperstructure;
  Turret mTurret;
  boolean flag = false;
  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = Shooter.getInstance();
    mSuperstructure = Superstructure.getInstance();
    mTurret = Turret.getInstance();
    mIntake = Intake.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSuperstructure.setWantedState(WantedState.SHOOT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mShooter.getIsReady()){
      flag = true;
    }
    if(mTurret.getOnTarget() && flag){
      mSuperstructure.shoot(true);
    }else{
      mSuperstructure.shoot(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSuperstructure.setWantedState(WantedState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//readd cell counter later.
  }
}
