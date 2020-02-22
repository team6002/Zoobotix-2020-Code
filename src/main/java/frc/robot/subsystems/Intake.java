/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.*;

import frc.lib.util.LatchedBoolean;

/**
 * Subsystem involved with intaking power cells in.
 */
public class Intake extends Subsystem {
  private static Intake mInstance;

  public static Intake getInstance(){
    if(mInstance == null){
      mInstance = new Intake();
    }
    return mInstance;
  }

  //motors  
  CANSparkMax mTopIntake = new CANSparkMax(Constants.kTopIntake, MotorType.kBrushless);
  CANSparkMax mGateIntake = new CANSparkMax(Constants.kGateIntake, MotorType.kBrushless);
  CANSparkMax mBotIntake = new CANSparkMax(Constants.kBotIntake, MotorType.kBrushless);
  VictorSPX mDeployIntake = new VictorSPX(Constants.kDeployIntake);

  //solenoids to deploy intake out
  Solenoid mOutDeploy = new Solenoid(Constants.kIntakeOut);
  Solenoid mDownDeploy = new Solenoid(Constants.kIntakeDown);
  
  //sensor 
  DigitalInput mCellSensor = new DigitalInput(Constants.kCellSensor);

  //encoders just because the sparks need them initalized
  CANEncoder mEncoderA = mTopIntake.getEncoder();
  CANEncoder mEncoderB = mGateIntake.getEncoder();
  CANEncoder mEncoderC = mBotIntake.getEncoder();

  //software states
  boolean isIntakeOn = false;
  boolean isGateOn = false;
  boolean isIntakeDeployed = false;
  boolean isIndexerFull = false;

  int cellCount = 0;
  LatchedBoolean cellEdge = new LatchedBoolean();
  
  //speed values
  //intaking balls
  double DEPLOYINTAKE_ON = 0;
  double TOPINTAKE_ON = 0.3;
  double BOTINTAKE_ON = 0.3;
  double GATE_IN = -0.70;

  //feed into the shooter
  double GATE_FEED = 0.3;
  double TOPINTAKE_FEED = 0.6;   
  double BOTINTAKE_FEED = -0.6;                                                                                                                                                                                                                        

  public Intake(){
    mTopIntake.restoreFactoryDefaults();
    mBotIntake.restoreFactoryDefaults();
    mGateIntake.restoreFactoryDefaults();
    mDeployIntake.configFactoryDefault();

    mTopIntake.setInverted(true);
    mBotIntake.setInverted(true);
    mGateIntake.setInverted(false);
    mDeployIntake.setInverted(false);

    mTopIntake.setIdleMode(IdleMode.kBrake);
    mBotIntake.setIdleMode(IdleMode.kBrake);
    mGateIntake.setIdleMode(IdleMode.kBrake);
    mDeployIntake.setNeutralMode(NeutralMode.Brake);

    mOutDeploy.set(false);
    mDownDeploy.set(false);

    //reset Encoders
    mEncoderA.setPosition(0);
    mEncoderB.setPosition(0);
    mEncoderC.setPosition(0);
    
  }

  private enum IntakeState{
    OFF,
    INTAKING,
    FEEDING_SHOOTER,
    REVERSE;
  }
  
  private IntakeState mIntakeState = IntakeState.OFF;

  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Intake.this) {

            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Intake.this) {
                switch(mIntakeState){
                  case OFF:
                    stop();
                    break;
                  case INTAKING:
                    // if switch, turn on the gate and store power cell
                    boolean detected = !mCellSensor.get();
                    // if(detected && cellCount == 4){
                    //   setOff();//store the fifth ball inside of the indexer mechanism.
                    // }else 
                    if(detected){
                      gateOn();
                      mTopIntake.set(0);
                    }else{
                      mTopIntake.set(TOPINTAKE_ON);
                      gateOff();
                    }

                    if(cellEdge.update(detected)){
                      cellCount++;
                    }

                    break;
                  case FEEDING_SHOOTER:
                    //if shooter is up to speed, set the gate to feed power cells in.
                    break;
                  default:
                    System.out.println("Unexpected intake state: " + mIntakeState);
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

  

  public void startIntaking(){
    if(mIntakeState != IntakeState.INTAKING){
      mIntakeState = IntakeState.INTAKING;
    }

    deployIntake();

    //gate is taken care of in loop
    mDeployIntake.set(ControlMode.PercentOutput, DEPLOYINTAKE_ON);
    
    mBotIntake.set(BOTINTAKE_ON);
    if(isGateOn){
      mTopIntake.set(0);
    }else{
      mTopIntake.set(TOPINTAKE_ON);
    }
    
  }

  public void gateOn(){
    isGateOn = true;
    mGateIntake.set(GATE_IN);
  }
  public void gateOff(){
    isGateOn = false;
    mGateIntake.set(0);
  }

  public void startFeeding(){
    if(mIntakeState != IntakeState.FEEDING_SHOOTER){
      mIntakeState = IntakeState.FEEDING_SHOOTER;
    }

    // stowIntake();
    mTopIntake.set(TOPINTAKE_FEED);
    mBotIntake.set(BOTINTAKE_FEED);
    mGateIntake.set(GATE_FEED);
  }

  public void deployIntake(){
    mOutDeploy.set(true);
    mDownDeploy.set(true);
    isIntakeDeployed = true;
  }
  public void stowIntake(){
    mDownDeploy.set(false);
    mOutDeploy.set(false);
    isIntakeDeployed = false;
  }

  public void setIntake(boolean pIntakeOn){
    if(pIntakeOn){
      mDeployIntake.set(ControlMode.PercentOutput, DEPLOYINTAKE_ON);
      isIntakeOn = true;
    }else{
      mDeployIntake.set(ControlMode.PercentOutput, 0);
      isIntakeOn = false;
    }
  }


  public void setOff(){
    if(mIntakeState != IntakeState.OFF){
      mIntakeState = IntakeState.OFF;
    }
    isGateOn = false;
    isIntakeOn = false;
    mTopIntake.set(0);
    mBotIntake.set(0);
    mGateIntake.set(0);
    mDeployIntake.set(ControlMode.PercentOutput, 0);
  }

  //if shooter detects a velocity drop, that means a power cell was shot.
  public void decreaseCellCount(){
    cellCount--;
  }

  public boolean isFull(){//or over capacity
    return cellCount >= 5;
  }

  public boolean isEmpty(){
    if(cellCount < 0) cellCount = 0;
    return cellCount <= 0;
  }

  public boolean getDetected(){
    return mCellSensor.get();
  }

  public boolean getIsGateOn(){
    return isGateOn;
  }
  public synchronized void stop(){
    setOff();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putString("Intake State", mIntakeState.toString());
    SmartDashboard.putNumber("Power Cells Stored", cellCount);
    SmartDashboard.putBoolean("Cell Sensor", mCellSensor.get());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
