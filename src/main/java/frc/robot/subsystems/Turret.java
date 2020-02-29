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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.*;
import frc.lib.util.Util;

/**
 * The part that the shooter sits on top of robot and spins.
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
  NetworkTable table = inst.getTable("SmartDashboard");
  NetworkTableEntry cXEntry, cYEntry;

  CANSparkMax mTurret = new CANSparkMax(Constants.kTurret, MotorType.kBrushless);
  int allowableError = 0;

  CANEncoder mEncoder = mTurret.getEncoder();
  CANPIDController mController = mTurret.getPIDController();

  //offset
  double kP = 0.00490;//proportional term for use in aiming the turret at the target
  double kI = 0.00003;
  double kD = 0.00003;
  PIDController mOffsetController = new PIDController(kP, kI, kD);


  //software states
  boolean onTarget = false;
  boolean wantTarget = false;

  public enum Hint{
    LEFT,
    RIGHT,
    NONE;
  }
  private Hint mHint = Hint.NONE;
  
  double setpoint = 0;
  double center = 80;
  double limitRight = 0;
  double limitLeft = 0;
  

  //speed constants
  double TURRET_LEFT = -0.01;
  double TURRET_RIGHT = 0.01;
  double TURRET_SWEEP_LEFT = -0.01;
  double TURRET_SWEEP_RIGHT = 0.01;


  public Turret(){
    mTurret.restoreFactoryDefaults();

    mTurret.setIdleMode(IdleMode.kBrake);
    mTurret.setInverted(false);

    //set pid constants.
    mController.setP(Constants.kTurretP);
    mController.setI(Constants.kTurretI);
    mController.setD(Constants.kTurretD);
    mController.setFF(Constants.kTurretF);
    mController.setIZone(Constants.kTurretIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    mOffsetController.reset();
    
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
                    if(wantTarget){
                      mControlState = ControlState.TARGETING;
                    }
                    setpoint = 0;
                    break;
                  case IDLING:
                    if(wantTarget){
                      mControlState = ControlState.TARGETING;
                    }
                    setpoint = 0;
                    mController.setReference(setpoint, ControlType.kDutyCycle);
                    break;
                  case TARGETING:
                    if(!wantTarget){
                      mControlState = ControlState.IDLING;
                    }
                    //check network tables for position of target
                    //move turret towards target
                    //report back when on target.
                    setpoint = updateTarget(getOffset());
                    
                    mController.setReference(setpoint, ControlType.kDutyCycle);
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

    mController.setReference(value, ControlType.kDutyCycle);
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
  public double getOffset(){//positive means off to the left.
    return center - readcX();
  }

  public double updateTarget(double offset){
    if(mControlState != ControlState.TARGETING){
      mControlState = ControlState.TARGETING;
    }
    
    double targetSetpoint = 0;
    if(readcX() == -1){//didn't find target
      onTarget = false;
      targetSetpoint = 0;
    }else{
      //given values from the network table, turn the turret towards the center of the vision target.
      if(Util.epsilonEquals(offset, 0, 5)){
        onTarget = true;
      }else{
        onTarget = false;
      }

      mOffsetController.setSetpoint(0);
      targetSetpoint = -mOffsetController.calculate(offset);
    }
    
    return targetSetpoint;
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
    // SmartDashboard.putString("hint", mHint.toString());
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("offset", getOffset());
    SmartDashboard.putBoolean("onTarget", getOnTarget());
    SmartDashboard.putNumber("Turret Position", mEncoder.getPosition());
    SmartDashboard.putString("Turret State", mControlState.toString());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
