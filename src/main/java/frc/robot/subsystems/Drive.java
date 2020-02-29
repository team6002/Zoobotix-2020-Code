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
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.*;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * The drive subsystem.  Controls the drivetrain on the robot.
 */

public class Drive extends SubsystemBase {
  private static Drive mInstance;

  public static Drive getInstance(){
    if(mInstance == null){
      mInstance = new Drive();
    }
    return mInstance;
  }

  //hardware
  CANSparkMax mLeftMaster = new CANSparkMax(Constants.kDriveLeftMaster, MotorType.kBrushless);
  CANSparkMax mRightMaster = new CANSparkMax(Constants.kDriveRightMaster, MotorType.kBrushless);
  CANSparkMax mLeftSlave = new CANSparkMax(Constants.kDriveLeftSlave, MotorType.kBrushless);
  CANSparkMax mRightSlave = new CANSparkMax(Constants.kDriveRightSlave, MotorType.kBrushless);
  Solenoid mShifter = new Solenoid(Constants.kShifter);

  CANEncoder mLeftEncoder = mLeftMaster.getEncoder();
  CANEncoder mRightEncoder = mRightMaster.getEncoder();
  CANEncoder mLeftSlaveEncoder = mLeftSlave.getEncoder();
  CANEncoder mRightSlaveEncoder = mRightSlave.getEncoder();
  
  

  DifferentialDrive mRoboDrive = new DifferentialDrive(mLeftMaster, mRightMaster);

  CANPIDController mLeftPIDController = mLeftMaster.getPIDController();
  CANPIDController mRightPIDController = mRightMaster.getPIDController();

  SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(Constants.ksVolts,
    Constants.kvVoltSecondsPerMeter,
    Constants.kaVoltSecondsSquaredPerMeter);

  public SimpleMotorFeedforward getFeedforward(){
    return mFeedforward;
  }

  AHRS mGyro = new AHRS(Port.kMXP);

  //kinematics and odometry
  DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
  DifferentialDriveOdometry mOdometry;

  Pose2d pose = new Pose2d();


  
  //hardware state
  private boolean mIsBrakeMode;

  // public void setMotor(double value){
  //   // mLeftSlave.set(value);
  //   // mLeftMaster.set(value);
  //   // mRightSlave.set(value);
  //   // mRightMaster.set(value);
  // }

  private Drive(){
    mLeftMaster.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mLeftSlave.restoreFactoryDefaults();
    mRightSlave.restoreFactoryDefaults();

    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    //invert motors
    mLeftMaster.setInverted(false);
    mRightMaster.setInverted(true);
    mLeftMaster.setOpenLoopRampRate(0.1);
    mRightMaster.setOpenLoopRampRate(0.1);

    //set up PIDControllers for the neos
    setDrivePIDF(mLeftPIDController);
    setDrivePIDF(mRightPIDController);

    //set brakemode
    mIsBrakeMode = false;
    setBrakeMode(false);

    mOdometry = new DifferentialDriveOdometry(getHeading());
    
    mShifter.set(false);
    //reset encoder and profile controller
    resetEncoders();
    resetOdometry();
  }

  private boolean isHighGear = false;
  public boolean isHighGear(){
    return isHighGear;
  }
  public void shift(boolean high){
    if(high){
      mShifter.set(true);
      isHighGear = true;
    }else{
      mShifter.set(false);
      isHighGear = false;
    }
  }

  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
    mLeftSlaveEncoder.setPosition(0);
    mRightSlaveEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance(){
    return(mLeftEncoder.getPosition() + mRightEncoder.getPosition())/2.0;
  }

  public void setMaxOutput(double maxOutput){
    mRoboDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    mGyro.reset();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(-mGyro.getAngle(), 360));
  }
  public Pose2d getPose(){
    return pose;
  }
  public DifferentialDriveKinematics getKinematics(){
    return mKinematics;
  }
  //TODO determine if we are running auto in low gear or high gear
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      mLeftEncoder.getVelocity() / Constants.kLowGearRatio * 2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches) / 60, 
      mRightEncoder.getVelocity() / Constants.kLowGearRatio * 2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches) / 60
    );
  }
  public void resetOdometry(){
    resetEncoders();
    mOdometry.resetPosition(pose, getHeading());
  }
  public CANPIDController getLeftPidController(){
    return mLeftPIDController;
  }
  public CANPIDController getRightPidController(){
    return mRightPIDController;
  }

  // public void setVoltage(double leftVolts, double rightVolts){
  //   mLeftMaster.set(leftVolts / 12);
  //   mRightMaster.set(rightVolts / 12);
  //   mRoboDrive.feed();
  // }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    mLeftPIDController.setReference(leftVolts, ControlType.kVoltage);
    mRightPIDController.setReference(rightVolts, ControlType.kVoltage);
    mRoboDrive.feed();
  }
  

  private enum ControlState {
    OPEN_LOOP,
    PROFILE; 
  }

  private ControlState mControlState = ControlState.OPEN_LOOP;

  
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
                // mOdometry.update(getHeading(), mLeftEncoder.getPosition(), mRightEncoder.getPosition());
                // pose = mOdometry.update(getHeading(), getWheelSpeeds());

                switch(mControlState){
                  case OPEN_LOOP:
                    break;
                  case PROFILE:
                    // if(!mTrapezoidController.atGoal()){
                    //   updateProfile();
                    // }

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
    mRoboDrive.arcadeDrive(turn, throttle, true);
  }

  public synchronized void setVelocity(double left, double right){
    if(mControlState != ControlState.PROFILE){
      mControlState = ControlState.PROFILE;
    }

    mLeftPIDController.setReference(left, ControlType.kVelocity);
    mRightPIDController.setReference(right, ControlType.kVelocity);
  }

  public synchronized void setBrakeMode(boolean enable){
    // if(mIsBrakeMode != enable){
    //   mIsBrakeMode = enable;
    //   IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      
    //   mRightMaster.setIdleMode(mode);
    //   mLeftMaster.setIdleMode(mode);
    //   mRightSlave.setIdleMode(mode);
    //   mLeftSlave.setIdleMode(mode);

    // }
    if(enable){
      mIsBrakeMode = true;
      mRightMaster.setIdleMode(IdleMode.kBrake);
      mLeftMaster.setIdleMode(IdleMode.kBrake);
      mRightSlave.setIdleMode(IdleMode.kBrake);
      mLeftSlave.setIdleMode(IdleMode.kBrake);
    }else{
      mIsBrakeMode = false;
      mRightMaster.setIdleMode(IdleMode.kCoast);
      mLeftMaster.setIdleMode(IdleMode.kCoast);
      mRightSlave.setIdleMode(IdleMode.kCoast);
      mLeftSlave.setIdleMode(IdleMode.kCoast);
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

  @Override
  public void periodic(){
    pose = mOdometry.update(
      getHeading(), 
      leftEncoderPositionMeters(), 
      rightEncoderPositionMeters()
    );
  }

  public double leftEncoderPositionMeters(){
    return mLeftEncoder.getPosition() / Constants.kLowGearRatio * 2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadiusInches);
  }
  public double rightEncoderPositionMeters(){
    return mRightEncoder.getPosition() / Constants.kLowGearRatio * 2 * Math.PI *  Units.inchesToMeters(Constants.kWheelRadiusInches);
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putNumber("leftEncoder Position Meters", leftEncoderPositionMeters());
    SmartDashboard.putNumber("rightEncoder Positon Meters", rightEncoderPositionMeters());
    SmartDashboard.putNumber("Left Side Wheel Speed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Side Wheel Speed", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putString("pose", mOdometry.getPoseMeters().toString());
    SmartDashboard.putBoolean("high gear", isHighGear());
  }

}
