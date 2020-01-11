/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * The drive subsystem.  Controls the drivetrain on the robot.
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

  //drive encoders.
  CANEncoder mLeftEncoder = mLeftMaster.getEncoder(EncoderType.kQuadrature, 4096);
  CANEncoder mRightEncoder = mRightMaster.getEncoder(EncoderType.kQuadrature, 4096);

  DifferentialDrive mRoboDrive = new DifferentialDrive(mLeftMaster, mRightMaster);

  //pid controller with trapezoid profile.
  ProfiledPIDController mController = new ProfiledPIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD,
                  Constants.kDriveConstraints, Constants.kDt);

  private final double kDegreePerInch = 360*(3.14/6); //don't know if this is still valid for pneumatics
  
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
                    updateProfile();
                    break;
                  default:
                    System.out.println("Unexpected drive control state: " + mControlState);
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
          stop();
        }
    });
  }

  public void setOpenLoop(double left, double right){
    mControlState = ControlState.OPEN_LOOP;
    mRightMaster.set(right);
    mLeftMaster.set(left);
  }

  public void arcadeDrive(double throttle, double turn){
    mRoboDrive.arcadeDrive(throttle, turn);
  }

  public void setGoal(int goal){
    mController.setGoal(goal);
  }

  public void updateProfile(){
    //have velocity follow the trapezoid pattern
    mController.calculate(mLeftEncoder.getVelocity());
  }

  void stop(){
    setOpenLoop(0, 0);
  }

}
