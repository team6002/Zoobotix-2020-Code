/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  CANSparkMax mWinchMaster = new CANSparkMax(Constants.kWinchMaster, MotorType.kBrushless);
  CANEncoder mEncoder = mWinchMaster.getEncoder();
  CANPIDController mController = mWinchMaster.getPIDController();
  TalonSRX mBalance = new TalonSRX(Constants.kBalance);

  Solenoid mClimberDeploy = new Solenoid(Constants.kClimberDeploy);
  Solenoid mWinchRatchet = new Solenoid(Constants.kWinchRatchet);




  public Climber(){
    mWinchMaster.restoreFactoryDefaults();
    mBalance.configFactoryDefault();

    //set winchmaster pidf
    mController.setP(Constants.kClimberP);
    mController.setI(Constants.kClimberI);
    mController.setD(Constants.kClimberD);
    mController.setFF(Constants.kClimberF);
    mController.setIZone(Constants.kClimberIz);
    mController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);

    mWinchMaster.setInverted(false);
    mBalance.setInverted(false);

    mWinchRatchet.set(false);
    mClimberDeploy.set(false);
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

  public void setWinch(double value){
    mController.setReference(value, ControlType.kDutyCycle);
  }

  public void setBalance(double value){
    mBalance.set(ControlMode.PercentOutput, value);
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putNumber("Climber Current", mWinchMaster.getOutputCurrent());

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
