/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * Subsystem for the hood shooter of lemons
 */
public class Shooter extends SubsystemBase {
  private static Shooter mInstance;

  public static Shooter getInstance(){
    if(mInstance == null){
      mInstance = new Shooter();
    }
    return mInstance;
  }

  //motors
  CANSparkMax mShooterMaster = new CANSparkMax(Constants.kShooterMaster, MotorType.kBrushless);
  CANSparkMax mShooterSlave = new CANSparkMax(Constants.kShooterSlave, MotorType.kBrushless);

  //encoder
  CANEncoder mEncoder = mShooterMaster.getEncoder();
  CANEncoder mSlaveEncoder = mShooterSlave.getEncoder();

  //pid controller
  CANPIDController mController = mShooterMaster.getPIDController();

  //variables
  boolean isReady = false;
  double setpoint = 0;
  boolean shotBall = false;

  //distance-rpm table NEEDS ACTUAL VALUES
  double[] distanceRPM = {2000, 2500, 2750};

  public Shooter(){
    mShooterMaster.restoreFactoryDefaults();
    mShooterSlave.restoreFactoryDefaults();

    mShooterSlave.follow(mShooterMaster, true);

    mShooterMaster.setInverted(true);
    
    
    mShooterMaster.setIdleMode(IdleMode.kCoast);
    mShooterSlave.setIdleMode(IdleMode.kCoast);

    //set up pid configs for the pid controller
    mController.setP(Constants.kShooterP);
    mController.setI(Constants.kShooterI);
    mController.setD(Constants.kShooterD);
    mController.setFF(Constants.kShooterF);
    mController.setIZone(Constants.kShooterIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    mController.setSmartMotionMaxAccel(Constants.kShooterMaxAccel, 0);
    mController.setSmartMotionMaxVelocity(Constants.kShooterMaxVel, 0);

    //reset encoder
    mEncoder.setPosition(0);
  }

  private enum ControlState {
    OPEN_LOOP,
    VELOCITY; 
  }

  private ControlState mControlState =  ControlState.OPEN_LOOP;

  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Shooter.this) {

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Shooter.this) {
                switch(mControlState){
                  case OPEN_LOOP:
                    shotBall = false;
                    break;
                  case VELOCITY:
                    if(Util.epsilonEquals(getVelocity(), setpoint, 150)){
                      isReady = true;
                    }else{
                      isReady = false;
                    }

                    //check if a ball was shot by seeing drops in velocity.
                    if(Math.abs(setpoint - getVelocity()) > 150){
                      shotBall = true;
                    }else{
                      shotBall = false;
                    }
                    break;
                  default:
                    System.out.println("Unexpected shooter control state: " + mControlState);
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

  public void setOpenLoop(double value){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }

    mController.setReference(value, ControlType.kDutyCycle);
  }

  public void setVelocity(double pWanted){
    if(mControlState != ControlState.VELOCITY){
      mControlState = ControlState.VELOCITY;
    }
    
    setpoint = pWanted;
    mController.setReference(pWanted, ControlType.kVelocity);

  }

  public double getRPM(double distance){//grab rpm from table
    if(distance >= 25){
      return distanceRPM[2];
    }else if(distance >= 10){
      return distanceRPM[1];
    }else{
      return distanceRPM[0];
    }
    
  }

  public boolean getShotBall(){
    return shotBall;
  }

  public boolean getIsReady(){
    return isReady;
  }

  public synchronized double getSetpoint(){
    return setpoint;
  }

  public synchronized double getVelocity(){
    return mEncoder.getVelocity();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putBoolean("Shot Ball?", shotBall);
    SmartDashboard.putBoolean("Shooter Ready", isReady);
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    SmartDashboard.putNumber("Shooter Master Current", mShooterMaster.getOutputCurrent());
    SmartDashboard.putNumber("Shooter Slave Current", mShooterSlave.getOutputCurrent());
    // SmartDashboard.putString("Shooter C.State", mControlState.toString());
  }

  public synchronized void stop(){
    setOpenLoop(0);
  }

  @Override
  public void periodic() {
  }
}
