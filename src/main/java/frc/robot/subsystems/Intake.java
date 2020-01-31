/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  private static Intake mInstance;

  public static Intake getInstance(){
    if(mInstance == null){
      mInstance = new Intake();
    }
    return mInstance;
  }

  // motor
  CANSparkMax mIntakeA = new CANSparkMax(Constants.kIntakeA, MotorType.kBrushless);
  CANSparkMax mIntakeB = new CANSparkMax(Constants.kIntakeB, MotorType.kBrushless);
  CANSparkMax mIntakeC = new CANSparkMax(Constants.kIntakeC, MotorType.kBrushless);
  CANSparkMax mIntakeD = new CANSparkMax(Constants.kIntakeD, MotorType.kBrushless);

  // encoders
  CANEncoder mEncoderA = mIntakeA.getEncoder();
  CANEncoder mEncoderB = mIntakeB.getEncoder();
  CANEncoder mEncoderC = mIntakeC.getEncoder();
  CANEncoder mEncoderD = mIntakeD.getEncoder();

  boolean isIntakeOn = false;
  boolean isIntakeDeployed = false;
  
  public Intake(){
    // reset motor
    mIntakeA.restoreFactoryDefaults();
    mIntakeB.restoreFactoryDefaults();
    mIntakeC.restoreFactoryDefaults();
    mIntakeD.restoreFactoryDefaults();

    // reset encoder
    mEncoderA.setPosition(0);
    mEncoderB.setPosition(0);
    mEncoderC.setPosition(0);
    mEncoderD.setPosition(0);
  }

  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
