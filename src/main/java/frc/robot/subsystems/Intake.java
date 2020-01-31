/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
  CANSparkMax mDeployIntake = new CANSparkMax(Constants.kDeployIntake, MotorType.kBrushless);



  //solenoids to deploy intake out
  Solenoid mOutDeploy = new Solenoid(Constants.kIntakeOut);
  Solenoid mDownDeploy = new Solenoid(Constants.kIntakeDown);
  
  //sensor 
  DigitalInput mCellSensor = new DigitalInput(Constants.kCellSensor);

  //encoders just because the sparks need them initalized
  CANEncoder mEncoderA = mTopIntake.getEncoder();
  CANEncoder mEncoderB = mGateIntake.getEncoder();
  CANEncoder mEncoderC = mBotIntake.getEncoder();
  CANEncoder mEncoderD = mDeployIntake.getEncoder();

  //software states
  boolean isIntakeOn = false;
  boolean isIntakeDeployed = false;
  boolean isIndexerFull = false;

  int cellCount = 0;
  LatchedBoolean cellEdge = new LatchedBoolean();
  
  //speed values
  //intaking balls
  double DEPLOYINTAKE_ON = 0;
  double TOPINTAKE_ON = 0;
  double BOTINTAKE_ON = 0;
  double GATE_IN = 0;

  //feed into the shooter
  double GATE_FEED = 0;
  double TOPINTAKE_FEED = 0;   
  double BOTINTAKE_FEED = 0;                                                                                                                                                                                                                        

  public Intake(){
    mTopIntake.restoreFactoryDefaults();
    mBotIntake.restoreFactoryDefaults();
    mGateIntake.restoreFactoryDefaults();
    mDeployIntake.restoreFactoryDefaults();

    mTopIntake.setInverted(false);
    mBotIntake.setInverted(false);
    mGateIntake.setInverted(false);
    mDeployIntake.setInverted(false);

    mTopIntake.setIdleMode(IdleMode.kBrake);
    mBotIntake.setIdleMode(IdleMode.kBrake);
    mGateIntake.setIdleMode(IdleMode.kBrake);
    mDeployIntake.setIdleMode(IdleMode.kBrake);

    mOutDeploy.set(false);
    mDownDeploy.set(false);

    //reset Encoders
    mEncoderA.setPosition(0);
    mEncoderB.setPosition(0);
    mEncoderC.setPosition(0);
    mEncoderD.setPosition(0);
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
                    break;
                  case INTAKING:
                    //if switch, turn on the gate and store power cell
                    boolean detected = cellEdge.update(mCellSensor.get());
                    if(detected && cellCount == 4){
                      setOff();//store the fifth ball inside of the indexer mechanism.
                    }else if(detected){
                      mGateIntake.set(GATE_IN);
                    }else{
                      mGateIntake.set(0);
                    }

                    if(detected){
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
    mDeployIntake.set(DEPLOYINTAKE_ON);
    mTopIntake.set(TOPINTAKE_ON);
    mBotIntake.set(BOTINTAKE_ON);
    
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
      mDeployIntake.set(DEPLOYINTAKE_ON);
      isIntakeOn = true;
    }else{
      mDeployIntake.set(0);
      isIntakeOn = false;
    }
  }


  public void setOff(){
    if(mIntakeState != IntakeState.OFF){
      mIntakeState = IntakeState.OFF;
    }

    mTopIntake.set(0);
    mBotIntake.set(0);
    mGateIntake.set(0);
    mDeployIntake.set(0);
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

  public synchronized void stop(){
    setOff();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putString("Intake State", mIntakeState.toString());
    SmartDashboard.putNumber("Power Cells", cellCount);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
