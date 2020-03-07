/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.security.DrbgParameters.NextBytes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * Subsystem that invovlves the winch and pneumatic deployed climbing arm
 */
public class Climber extends Subsystem {
  private static Climber mInstance;

  public static Climber getInstance(){
    if(mInstance == null){
      mInstance = new Climber();
    }
    return mInstance;
  }
  
  //hardware
  CANSparkMax mWinchMaster = new CANSparkMax(Constants.kWinchMaster, MotorType.kBrushless);
  CANEncoder mEncoder = mWinchMaster.getEncoder();
  CANPIDController mController = mWinchMaster.getPIDController();
  //TODO balance is now a neo
  // TalonSRX mBalance = new TalonSRX(Constants.kBalance);

  Solenoid mClimberDeploy = new Solenoid(Constants.kClimberDeploy);
  Solenoid mWinchRatchet = new Solenoid(Constants.kWinchRatchet);

  //software states
  double wantedPosition = 0;
  double winchPosition = mEncoder.getPosition();
  double offset = 0;

  //winch positions
  private double LEVEL = -120;
  private double HIGH = -145;
  private double HIGH_LOCK = -135;
  private double LOW_LOCK = -110;
  private double CLIMB = -50;
  private double MAX = -150;
  private double MIN = 0;

  public enum ClimberState {
    HOME,
    DEPLOYING,
    LEVEL_SWITCH,
    HIGH_SWITCH,
    LOW_LOCK_ON_BAR,
    HIGH_LOCK_ON_BAR,
    PISTON_RELEASED,
    CLIMBING
  }

  private ClimberState mClimberState = ClimberState.HOME;

  public enum ClimberWantedState {
    NEXT,
    IDLE,
    BACK,
    WANT_HIGHER_SWITCH,
    RELEASE_PISTON,
  }

  private ClimberWantedState mClimberWantedState = ClimberWantedState.IDLE;                                                                                                                                                                      


  public Climber(){
    mWinchMaster.restoreFactoryDefaults();
    // mBalance.configFactoryDefault();

    //set winchmaster pidf
    mController.setP(Constants.kClimberP);
    mController.setI(Constants.kClimberI);
    mController.setD(Constants.kClimberD);
    mController.setFF(Constants.kClimberF);
    mController.setIZone(Constants.kClimberIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    mController.setSmartMotionMaxAccel(Constants.kClimberMaxAccel, 0);
    mController.setSmartMotionMaxVelocity(Constants.kClimberMaxVel, 0);

    mWinchMaster.setInverted(false);
    // mBalance.setInverted(false);

    mWinchRatchet.set(false);
    mClimberDeploy.set(false);
    releaseWinch();

    mEncoder.setPosition(0);
  }

  private boolean mStateChanged = false;

  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Climber.this) {
              mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
          setWinch(wantedPosition + offset);

          //controlling state of climnber
          ClimberState newState = mClimberState;
          synchronized (Climber.this) {
            switch(mClimberState) {
              case HOME:
                newState = handleHome();
                break;
              case DEPLOYING:
                newState = handleDeploying();
                break;
              case LEVEL_SWITCH:
                newState = handleLevelSwitch();
                break;
              case HIGH_SWITCH:
                newState = handleHighSwitch();
                break;
              case LOW_LOCK_ON_BAR:
                newState = handleLowLockOnBar();
                break;
              case HIGH_LOCK_ON_BAR:
                newState = handleHighLockOnBar();
                break;
              case PISTON_RELEASED:
                newState = handlePistonReleased();
                break;
              case CLIMBING:
                newState = handleClimbing();
                break;
              default:
                newState = mClimberState;
                break;
            }
            
            if(newState != mClimberState){
              System.out.println("Climber state " + mClimberState + " to " + newState + " TimeStamp: " + Timer.getFPGATimestamp());
              mClimberState = newState;
              mClimberWantedState = ClimberWantedState.IDLE;
              mStateChanged = true;
            }else{
              mStateChanged = false;
            }
          }
        }

        @Override
        public void onStop(double timestamp) {
          stop();
        }
    });
  }

  private ClimberState handleHome(){

    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.HOME;
      case NEXT:
        releaseWinch();
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.DEPLOYING;
      default:
        return ClimberState.HOME;
    }
  }

  private ClimberState handleDeploying(){
    if(mStateChanged){
      releaseWinch();
      setClimberDeploy(true);
      setWinchSetpoint(LEVEL);
      mClimberWantedState = ClimberWantedState.IDLE;
    }

    if(Util.epsilonEquals(mEncoder.getPosition(), LEVEL, 5)){
      return ClimberState.LEVEL_SWITCH;
    }else{
      return ClimberState.DEPLOYING;
    }
  }

  private ClimberState handleLevelSwitch(){
    if(mStateChanged) mClimberWantedState = ClimberWantedState.IDLE;

    setWinchSetpoint(LEVEL);

    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.LEVEL_SWITCH;
      case NEXT:
        setWinchSetpoint(LOW_LOCK);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.LOW_LOCK_ON_BAR;
      case WANT_HIGHER_SWITCH:
        setWinchSetpoint(HIGH);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.HIGH_SWITCH;
      case RELEASE_PISTON:
        setClimberDeploy(false);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.PISTON_RELEASED;
      default:
        return ClimberState.LEVEL_SWITCH;
    }
  }

  private ClimberState handleHighSwitch(){
    setWinchSetpoint(HIGH);
    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.HIGH_SWITCH;
      case NEXT:
        setWinchSetpoint(HIGH_LOCK);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.HIGH_LOCK_ON_BAR;
      case BACK:
        setWinchSetpoint(LEVEL);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.LEVEL_SWITCH;
      case RELEASE_PISTON:
        setClimberDeploy(false);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.PISTON_RELEASED;
      default:
        return ClimberState.HIGH_SWITCH;
    }
  }

  private ClimberState handleLowLockOnBar(){
    if(mStateChanged) mClimberWantedState = ClimberWantedState.IDLE;

    setWinchSetpoint(LOW_LOCK);
    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.LOW_LOCK_ON_BAR;
      case NEXT: 
        setClimberDeploy(false);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.PISTON_RELEASED;
      case BACK:
        setWinchSetpoint(LEVEL);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.LEVEL_SWITCH;
      default:
        return ClimberState.LOW_LOCK_ON_BAR;
    }
  }

  private ClimberState handleHighLockOnBar(){
    if(mStateChanged) mClimberWantedState = ClimberWantedState.IDLE;

    setWinchSetpoint(HIGH_LOCK);
    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.HIGH_LOCK_ON_BAR;
      case NEXT:
        setClimberDeploy(false);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.PISTON_RELEASED;
      case BACK:
        setWinchSetpoint(HIGH_LOCK);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.HIGH_SWITCH;
      default:
        return ClimberState.HIGH_LOCK_ON_BAR;
    }
  }

  private ClimberState handlePistonReleased(){
    if(mStateChanged) mClimberWantedState = ClimberWantedState.IDLE;

    setClimberDeploy(false);
    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.PISTON_RELEASED;
      case NEXT:
        setWinchSetpoint(CLIMB);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.CLIMBING;
      default:
        return ClimberState.PISTON_RELEASED;
    }
  }

  private ClimberState handleClimbing(){
    setWinchSetpoint(CLIMB);
    switch(mClimberWantedState){
      case IDLE:
        return ClimberState.CLIMBING;
      case NEXT:
        setWinchSetpoint(0);
        mClimberWantedState = ClimberWantedState.IDLE;
        return ClimberState.HOME;
      default:
        return ClimberState.CLIMBING;
    }
  }

  public void setWantedState(ClimberWantedState wantedState){
    mClimberWantedState = wantedState;
  }

  //winch is default engaged for power loss.
  boolean isWinchEngaged = true;
  public void releaseWinch(){
    mWinchRatchet.set(true);
    isWinchEngaged = false;
  }
  public void engageWinch(){
    mWinchRatchet.set(false);
    isWinchEngaged = true;
  }

  boolean isClimberDeployed = false;
  public void setClimberDeploy(boolean on){
    mClimberDeploy.set(on);
    isClimberDeployed = on;
  }

  public boolean getIsClimberDeployed(){
    return isClimberDeployed;
  }
  
  public boolean getIsWinchEngaged(){
    return isWinchEngaged;
  }

  public void setWinchSetpoint(double setpoint){
    // mController.setReference(setpoint, ControlType.kSmartMotion);
    wantedPosition = setpoint;
  }

  public void increaseOffset(){
    offset = offset - 1; 
  }

  public void decreaseOffset(){
    offset = offset + 1;
  }

  public void setWinch(double pWantedPosition){
    if(pWantedPosition < MAX){
      pWantedPosition = MAX;
    }else if(pWantedPosition > MIN){
      pWantedPosition = MIN;
    }
    mController.setReference(pWantedPosition, ControlType.kPosition);
  }

  public void setBalance(double value){
    // mBalance.set(ControlMode.PercentOutput, value);
  }

  public void resetEncoder(){
    mEncoder.setPosition(0);
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putNumber("Climber Current", mWinchMaster.getOutputCurrent());
    // SmartDashboard.putNumber("Climber Volts", mWinchMaster.getBusVoltage());
    SmartDashboard.putNumber("Climber Given Volts", mWinchMaster.get());
    SmartDashboard.putNumber("Climber Position", mEncoder.getPosition());
    SmartDashboard.putString("Climber State", mClimberState.toString());
    SmartDashboard.putString("Climber Wanted State", mClimberWantedState.toString());
    SmartDashboard.putNumber("Climber Wanted Position", wantedPosition);
    SmartDashboard.putNumber("Climber Offset", offset);
  }

  public void stop(){
    mController.setReference(0, ControlType.kDutyCycle);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
