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
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.auto.AutoTrajectories;
import frc.robot.auto.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Super8BallAuto extends SequentialCommandGroup {
  /**
   * Starts on the left side, then picks up the two balls from opposing color's trench.
   * Runs back towards the center and shoots the 5.
   * Grabs the 3 balls from the rendezvous point.
   * And then shoots those 3 at the optimal place.
   */

  static Drive mDrive = Drive.getInstance();
  public Super8BallAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      //parallel command to start intake and drive backwards
      new ParallelCommandGroup(//drive back to shooting position.
        new RevFlywheel(2300),
        driveTrajectory(new AutoTrajectories().OppositeTrenchToShootingPosition())
        //stop intaking
      ),
      new ShootCommand().withTimeout(3)

      //grab the 4 or so balls on the rendevouz point
      // new ParallelCommandGroup(
      //   new StartIntakeCommand(),
      //   driveTrajectory(trajectory)
      // ),
      //back to shooting position and fire.
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
    return "Super 8 Ball Auto";
  }
}
