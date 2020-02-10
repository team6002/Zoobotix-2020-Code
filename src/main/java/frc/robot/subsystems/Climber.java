/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Constants;

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
  TalonSRX mWinchMaster = new TalonSRX(Constants.kWinchMaster);
  TalonSRX mWinchSlave = new TalonSRX(Constants.kWinchSlave);
  TalonSRX mBalance = new TalonSRX(Constants.kBalance);
  Solenoid mWinchRatchet = new Solenoid(Constants.kWinchRatchet);


  public Climber(){
    mWinchMaster.configFactoryDefault();
    mWinchSlave.configFactoryDefault();
    mBalance.configFactoryDefault();

    mWinchSlave.set(ControlMode.Follower, Constants.kWinchMaster);

    mWinchMaster.setInverted(false);
    mWinchSlave.setInverted(false);
    mBalance.setInverted(false);

    mWinchMaster.setSensorPhase(false);
    mWinchSlave.setSensorPhase(false);

    mWinchRatchet.set(false);
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
  
  public boolean getIsWinchEngaged(){
    return isWinchEngaged;
  }

  public void setWinch(double value){
    mWinchMaster.set(ControlMode.PercentOutput, value);
  }

  public void setBalance(double value){
    mBalance.set(ControlMode.PercentOutput, value);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
