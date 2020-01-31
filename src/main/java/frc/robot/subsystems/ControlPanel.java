/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ControlPanel extends Subsystem {
  private static ControlPanel mInstance;

  public static ControlPanel getInstance(){
    if(mInstance == null){
      mInstance = new ControlPanel();
    }
    return mInstance;
  }

  // motor 
  TalonSRX mControlPanel = new TalonSRX(Constants.kControlPanel);
  Solenoid mDeploy =  new Solenoid(Constants.kControlPanelSolenoid);

  public ControlPanel(){
    // reset motor
    mControlPanel.configFactoryDefault();

    // set mode to brake
    mControlPanel.setNeutralMode(NeutralMode.Brake);

    //set solenoid
    mDeploy.set(false);
  }

  // objects
  public void deployControlPanel(){
    mDeploy.set(true);
  }  

  public void stowControlPanel(){
    mDeploy.set(false);
  }
  
  public void setOpenLoop(double value){
    mControlPanel.set(ControlMode.PercentOutput, value);
  }

  public void setRotation(double targetPosition){
    mControlPanel.set(ControlMode.Position, targetPosition);
  }

  // accessor
  private boolean controlPanelDeployed = false;

  public boolean getControlPanelDeployed(){
    return controlPanelDeployed;
  }

  

   
  // no touchy touchy
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
