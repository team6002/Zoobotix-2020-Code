/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

/*
command that stops the intake instantly.
*/
public class StopIntakeCommand extends InstantCommand {
  Intake mIntake;
  Superstructure mSuperstructure;
  public StopIntakeCommand() {
    mIntake = Intake.getInstance();
    mSuperstructure = Superstructure.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSuperstructure.setWantedState(WantedState.IDLE);
  }
}
