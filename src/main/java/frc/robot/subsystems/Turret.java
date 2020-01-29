/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.*;
import frc.lib.util.Util;

/**
 * The part that the shooter sits on top of and spins.
 */
public class Turret extends Subsystem {
  private static Turret mInstance;

  public static Turret getInstance(){
    if(mInstance == null){
      mInstance = new Turret();
    }
    return mInstance;
  }

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("Smartdashboard");
  NetworkTableEntry cXEntry, cYEntry;

  TalonSRX mTurret = new TalonSRX(Constants.kTurret);
  int allowableError = 0;

  //software states
  boolean onTarget = false;
  boolean wantTarget = false;

  public enum Hint{
    LEFT,
    RIGHT,
    NONE;
  }
  private Hint mHint = Hint.NONE;

  //speed constants
  double TURRET_LEFT = -0.1;
  double TURRET_RIGHT = 0.1;
  double TURRET_SWEEP_LEFT = -0.1;
  double TURRET_SWEEP_RIGHT = 0.1;


  public Turret(){
    mTurret.set(ControlMode.PercentOutput, 0);
    mTurret.configFactoryDefault();

    mTurret.setNeutralMode(NeutralMode.Brake);
    mTurret.setInverted(false);

    //encoder
    mTurret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

    //set pid constants.
    mTurret.config_kP(0, Constants.kTurretP);
    mTurret.config_kI(0, Constants.kTurretI);
    mTurret.config_kD(0, Constants.kTurretD);
    mTurret.config_kF(0, Constants.kTurretF);
    
    mTurret.configAllowableClosedloopError(0, allowableError, 30);
    
    // instead of reseting encoder, might have to reference from absolute encoder.

  }

  private enum ControlState{
    OPEN_LOOP,
    IDLING,
    TARGETING;
  }

  private ControlState mControlState = ControlState.OPEN_LOOP;

  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Turret.this) {

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Turret.this) {
                switch(mControlState){
                  case OPEN_LOOP:
                    break;
                  case IDLING:
                    if(wantTarget){
                      mControlState = ControlState.TARGETING;
                    }
                    break;
                  case TARGETING:
                    if(!wantTarget){
                      mControlState = ControlState.IDLING;
                    }
                    //check network tables for position of target
                    //move turret towards target
                    //report back when on target.
                    
                    break;
                  default:
                    System.out.println("Unexpected turret control state: " + mControlState);
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

    mTurret.set(ControlMode.PercentOutput, value);
  }

  public void setTurretState(boolean wantOn){
    if(wantOn) wantTarget = true;
    else wantTarget = false;
  }

  public boolean getWantTarget(){
    return wantTarget;
  }

  //read network tables
  public double readcX(){
    return table.getEntry("cX").getDouble(-1);
  }
  public double readcY(){
    return table.getEntry("cY").getDouble(-1);
  }
  public double readCenter(){//should never be -1
    return table.getEntry("center").getDouble(-1);
  }
  public double getOffset(){
    return readCenter() - readcX();
  }

  public void updateTarget(double offset){
    if(mControlState != ControlState.TARGETING){
      mControlState = ControlState.TARGETING;
    }
    
    if(readcX() == -1){//didn't find target
      onTarget = false;
      if(mHint == Hint.LEFT){
        mTurret.set(ControlMode.PercentOutput, TURRET_SWEEP_LEFT);
      }else if(mHint == Hint.RIGHT){
        mTurret.set(ControlMode.PercentOutput, TURRET_SWEEP_RIGHT);
      }else{
        mTurret.set(ControlMode.PercentOutput, 0);
      }

    }else{
      //given values from the network table, turn the turret towards the center of the vision target.
      if(Util.epsilonEquals(offset, 0, 10)){
        onTarget = true;
        mTurret.set(ControlMode.PercentOutput, 0);
      }else if(offset > 0){
        onTarget = false;
        mTurret.set(ControlMode.PercentOutput, TURRET_LEFT);
      }else{
        onTarget = false;
        mTurret.set(ControlMode.PercentOutput, TURRET_RIGHT);
      }
    }
    
  }

  public void setHint(Hint hint){
    mHint = hint;
  }
  

  public boolean getOnTarget(){
    return onTarget;
  }

  public synchronized void stop(){
    setOpenLoop(0);
  }

  public void outputToSmartdashboard(){
    SmartDashboard.putString("hint", mHint.toString());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
