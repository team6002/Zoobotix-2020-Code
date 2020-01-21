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

/**
 * Add your docs here.
 */
public class Indexer extends Subsystem {
  private static Indexer mInstance;

  public static Indexer getInstance(){
    if(mInstance == null){
      mInstance = new Indexer();
    }
    return mInstance;
  }

  //motor
  CANSparkMax mIndexer = new CANSparkMax(Constants.kIndexer, MotorType.kBrushless);

  //controller
  CANPIDController mController = mIndexer.getPIDController();

  //encoder
  CANEncoder mIndexerEncoder = mIndexer.getEncoder(EncoderType.kQuadrature, 4096);

  public Indexer(){
    mIndexer.restoreFactoryDefaults();

    //set up pid configs for the pid controller
    mController.setP(Constants.kIndexerP);
    mController.setI(Constants.kIndexerI);
    mController.setD(Constants.kIndexerD);
    mController.setIZone(Constants.kIndexerIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    mController.setSmartMotionMaxAccel(Constants.kIndexerMaxAccel, 0);
    mController.setSmartMotionMaxVelocity(Constants.kIndexerMaxVel, 0);

    mIndexer.setInverted(false);
    mIndexer.setIdleMode(IdleMode.kCoast);

    mIndexerEncoder.setPosition(0);
  }

  private enum ControlState {
    OPEN_LOOP,
    VELOCITY; 
  }

  private ControlState mControlState =  ControlState.OPEN_LOOP;

  public void setOpenLoop(double value){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    mController.setReference(value, ControlType.kVoltage);
  }

  public void setVelocity(double setpoint){
    if(mControlState != ControlState.VELOCITY){
      mControlState = ControlState.VELOCITY;
    }

    mController.setReference(setpoint, ControlType.kSmartVelocity);
  }



  public double getVelocity(){
    return mIndexerEncoder.getVelocity();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putNumber("Indexer Velocity", getVelocity());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
