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
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * Subsystem for the hood shooter of lemons
 */
public class Shooter extends Subsystem {
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
  CANEncoder mShooterEncoder = mShooterMaster.getEncoder();
  CANEncoder mShooterSlaveEncoder = mShooterSlave.getEncoder();

  //pid controller
  CANPIDController mController = mShooterMaster.getPIDController();

  public Shooter(){
    mShooterMaster.restoreFactoryDefaults();
    mShooterSlave.restoreFactoryDefaults();

    // mShooterSlave.follow(mShooterMaster, true);
    mShooterSlave.follow(mShooterMaster, true);

    //set up pid configs for the pid controller
    mController.setP(Constants.kShooterP);
    mController.setI(Constants.kShooterI);
    mController.setD(Constants.kShooterD);
    mController.setFF(Constants.kShooterF);
    mController.setIZone(Constants.kShooterIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    mController.setSmartMotionMaxAccel(Constants.kShooterMaxAccel, 0);
    mController.setSmartMotionMaxVelocity(Constants.kShooterMaxVel, 0);

    mShooterMaster.setInverted(false);
    // mShooterSlave.setInverted(false);
    mShooterMaster.setIdleMode(IdleMode.kCoast);
    mShooterSlave.setIdleMode(IdleMode.kCoast);

    mShooterEncoder.setPosition(0);
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
                    break;
                  case VELOCITY:
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

  public void setVelocity(double setpoint){
    if(mControlState != ControlState.VELOCITY){
      mControlState = ControlState.VELOCITY;
    }

    mController.setReference(setpoint, ControlType.kVelocity);

  }

  public synchronized double getVelocity(){
    return mShooterEncoder.getVelocity();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putNumber("Shooter Velocity", getVelocity());
    SmartDashboard.putNumber("Master Shooter Value", mShooterMaster.getOutputCurrent());
    SmartDashboard.putNumber("Slave Shooter Value", mShooterSlave.getOutputCurrent());
    SmartDashboard.putString("Shooter C.State", mControlState.toString());
  }

  private void stop(){
    setOpenLoop(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
