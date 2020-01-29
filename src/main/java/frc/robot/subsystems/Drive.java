/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.EncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  ProfiledPIDController mTrapezoidController = new ProfiledPIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD, Constants.kDriveConstraints, Constants.kDt);
  
  //pid controller for the neos.
  CANPIDController mLeftPIDController = mLeftMaster.getPIDController();
  CANPIDController mRightPIDController = mRightMaster.getPIDController();
  
  //hardware state
  private boolean mIsBrakeMode;

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

    //set up PIDControllers for the neos
    setDrivePIDF(mLeftPIDController);
    setDrivePIDF(mRightPIDController);

    //set brakemode
    mIsBrakeMode = false;
    setBrakeMode(false);

    //reset encoder and profile controller
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
    mTrapezoidController.reset(mLeftEncoder.getPosition());
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
                    if(!mTrapezoidController.atGoal()){
                      updateProfile();
                    }

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
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    mRightMaster.set(right);
    mLeftMaster.set(left);
  }

  public void arcadeDrive(double throttle, double turn){
    mRoboDrive.arcadeDrive(throttle, turn);
  }

  public void setGoal(int goal){
    if(mControlState != ControlState.PROFILE){
      mControlState = ControlState.PROFILE;
    }
    mTrapezoidController.setGoal(goal);
  }

  public void updateProfile(){
    //have velocity follow the trapezoid pattern
    double left = mTrapezoidController.calculate(mLeftEncoder.getVelocity());
    double right = mTrapezoidController.calculate(mRightEncoder.getVelocity());
    setVelocity(left, right);
  }

  public boolean finishedGoal(){
    return mTrapezoidController.atSetpoint();
  }

  public synchronized void setVelocity(double left, double right){
    if(mControlState != ControlState.PROFILE){
      mControlState = ControlState.PROFILE;
    }

    mLeftPIDController.setReference(left, ControlType.kVelocity);
    mRightPIDController.setReference(right, ControlType.kVelocity);
  }

  public synchronized void setBrakeMode(boolean enable){
    if(mIsBrakeMode != enable){
      mIsBrakeMode = enable;
      IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      
      mRightMaster.setIdleMode(mode);
      mLeftMaster.setIdleMode(mode);
      mRightSlave.setIdleMode(mode);
      mLeftSlave.setIdleMode(mode);

    }
  }

  public boolean isBrakeMode(){
    return mIsBrakeMode;
  }

  public synchronized void stop(){
    setOpenLoop(0, 0);
  }

  //constants are in the constants files
  public void setDrivePIDF(CANPIDController controller){
    controller.setP(Constants.kNeoDriveP);
    controller.setI(Constants.kNeoDriveI);
    controller.setD(Constants.kNeoDriveD);
    controller.setIZone(Constants.kNeoDriveIz);
    controller.setFF(Constants.kNeoDriveF);
    controller.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
  }

  public void outputToSmartDashboard(){
  }

}
