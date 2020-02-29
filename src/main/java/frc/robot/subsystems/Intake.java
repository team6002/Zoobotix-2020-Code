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
import edu.wpi.first.wpilibj.Timer;
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

  
  
  //hardware
  CANSparkMax mTopIntake = new CANSparkMax(Constants.kTopIntake, MotorType.kBrushless);
  CANSparkMax mGateIntake = new CANSparkMax(Constants.kGateIntake, MotorType.kBrushless);
  CANSparkMax mBotIntake = new CANSparkMax(Constants.kBotIntake, MotorType.kBrushless);
  // VictorSPX mDeployIntake = new VictorSPX(Constants.kDeployIntake);

  Solenoid mOutDeploy = new Solenoid(Constants.kIntakeOut);
  Solenoid mDownDeploy = new Solenoid(Constants.kIntakeDown);

  DigitalInput mCellSensor = new DigitalInput(Constants.kCellSensor);
  DigitalInput mEmptySensor = new DigitalInput(Constants.kEmptySensor);
  DigitalInput mFullSensor = new DigitalInput(Constants.kFullSensor);

  Timer timer = new Timer();

  //encoders just because the sparks need them initalized
  CANEncoder mEncoderA = mTopIntake.getEncoder();
  CANEncoder mEncoderB = mGateIntake.getEncoder();
  CANEncoder mEncoderC = mBotIntake.getEncoder();

  //software states
  boolean isIntakeOn = false;
  boolean isGateOn = false;
  boolean isIntakeDeployed = false;
  boolean isIndexerFull = false;
  boolean flag = false;

  double startTime = 0;
  int cellCount = 3; //we start with 3 in autonomous
  LatchedBoolean cellEdge = new LatchedBoolean();
  
  //speed values
  //intaking balls
  double DEPLOYINTAKE_ON = 0;
  double TOPINTAKE_ON = 0.3;
  double BOTINTAKE_ON = 0.3;
  double GATE_IN = -0.25;

  //feed into the shooter
  double GATE_FEED = 0.5;
  double TOPINTAKE_FEED = 0.5;   
  double BOTINTAKE_FEED = -0.5;
  
  //eject
  double GATE_EJECT = -0.1;
  double TOPINTAKE_EJECT = -0.2;
  double BOTINTAKE_EJECT = -0.2;

  public Intake(){
    mTopIntake.restoreFactoryDefaults();
    mBotIntake.restoreFactoryDefaults();
    mGateIntake.restoreFactoryDefaults();
    // mDeployIntake.configFactoryDefault();

    mTopIntake.setInverted(true);
    mBotIntake.setInverted(true);
    mGateIntake.setInverted(false);
    // mDeployIntake.setInverted(false);

    mTopIntake.setIdleMode(IdleMode.kBrake);
    mBotIntake.setIdleMode(IdleMode.kBrake);
    mGateIntake.setIdleMode(IdleMode.kBrake);
    // mDeployIntake.setNeutralMode(NeutralMode.Brake);

    mOutDeploy.set(false);
    mDownDeploy.set(false);

    timer.start();

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

  private enum IndexerState{
    EMPTY,
    PARTIAL_FILL,
    FULL,
  }

  private IndexerState mIndexerState = IndexerState.EMPTY;
  
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

              boolean detected = mCellSensor.get();
              boolean isFull = mFullSensor.get();
                switch(mIntakeState){
                  case OFF:
                    stop();
                    break;
                  case INTAKING:
                    switch(mIndexerState){
                      case EMPTY:
                        if(detected){
                          mIndexerState = IndexerState.PARTIAL_FILL;
                          gateOn();
                          mTopIntake.set(0);
                        }else{
                          gateOff();
                          mTopIntake.set(TOPINTAKE_ON);
                        }
                        break;
                      case PARTIAL_FILL:
                        if(isFull){
                          if(!flag){
                            startTime = timer.getFPGATimestamp();
                            flag = true;
                          }
                          if(timer.getFPGATimestamp() > startTime + 0.075){
                            gateOff();
                            mTopIntake.set(TOPINTAKE_ON);
                            flag = false;
                            mIndexerState = IndexerState.FULL;
                          }else{
                            gateOn();
                            mTopIntake.set(0);
                          }
                        }else {
                          if(detected){
                            mIndexerState = IndexerState.PARTIAL_FILL;
                            gateOn();
                            mTopIntake.set(0);
                          }else{
                            gateOff();
                            mTopIntake.set(TOPINTAKE_ON);
                          }
                        }
                        break;
                      case FULL:
                        if(!isFull){
                          if(mEmptySensor.get()){
                            mIndexerState = IndexerState.EMPTY;
                          }else{
                            mIndexerState = IndexerState.PARTIAL_FILL;
                          }
                        }
                        if(detected){
                          setOff();
                        }else{
                         mTopIntake.set(TOPINTAKE_ON);
                        }
                        break;
                    }
                    
                    

                    if(cellEdge.update(detected)){
                      cellCount++;
                    }

                    break;
                  case FEEDING_SHOOTER:
                    if(cellEdge.update(detected)){
                    cellCount--;
                    }
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

    // deployIntake();

    //gate is taken care of in loop
    // mDeployIntake.set(ControlMode.PercentOutput, DEPLOYINTAKE_ON);
    
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

  public void eject(){
   if(mIntakeState != IntakeState.REVERSE){
     mIntakeState = IntakeState.REVERSE;
   }

   mTopIntake.set(TOPINTAKE_EJECT);
   mBotIntake.set(BOTINTAKE_EJECT);
   mGateIntake.set(GATE_EJECT);
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
      // mDeployIntake.set(ControlMode.PercentOutput, DEPLOYINTAKE_ON);
      isIntakeOn = true;
    }else{
      // mDeployIntake.set(ControlMode.PercentOutput, 0);
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
    // mDeployIntake.set(ControlMode.PercentOutput, 0);
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
    timer.stop();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putString("Intake State", mIntakeState.toString());
    SmartDashboard.putString("Indexer State", mIndexerState.toString());
    SmartDashboard.putNumber("Power Cells Stored", cellCount);
    SmartDashboard.putBoolean("Cell Sensor", mCellSensor.get());
    SmartDashboard.putBoolean("Empty Sensor", mEmptySensor.get());
    SmartDashboard.putBoolean("Full Sensor", mFullSensor.get());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
