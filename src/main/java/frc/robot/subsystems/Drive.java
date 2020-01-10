/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * The drive subsystem.  Controls the drivetrain on teh robot.
 */
public class Drive extends Subsystem {
  private static Drive mInstance;

  public static Drive getInstance(){
    if(mInstance == null){
      mInstance = new Drive();
    }
    return mInstance;
  }
  //motors
  CANSparkMax mLeftMaster = new CANSparkMax(Constants.kDriveLeftMaster, MotorType.kBrushless);
  CANSparkMax mRightMaster = new CANSparkMax(Constants.kDriveRightMaster, MotorType.kBrushless);
  CANSparkMax mLeftSlave = new CANSparkMax(Constants.kDriveLeftSlave, MotorType.kBrushless);
  CANSparkMax mRightSlave = new CANSparkMax(Constants.kDriveRightSlave, MotorType.kBrushless);


  
  private Drive(){
    mLeftMaster.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mLeftSlave.restoreFactoryDefaults();
    mRightSlave.restoreFactoryDefaults();

    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    //invert motors
    mLeftMaster.setInverted(true);
    mRightMaster.setInverted(false);
    mLeftMaster.setOpenLoopRampRate(0.1);
    mRightMaster.setOpenLoopRampRate(0.1);
    

  }
  Solenoid mShifter = new Solenoid(Constants.kShifter);

  private enum ControlState {
    OPEN_LOOP,
    PROFILE; 
  }

  private ControlState mControlState = ControlState.OPEN_LOOP;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch(mControlState){
                  case OPEN_LOOP:
                    break;
                  case PROFILE:
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    });
  }

  public void setOpenLoop(double left, double right){
    mRightMaster.set(right);
    mLeftMaster.set(left);
  }

  public void arcadeDrive(double throttle, double turn){
    setOpenLoop(throttle + turn, throttle - turn);
  }
  // hello testing 123
}
