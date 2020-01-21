/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Conveyor extends Subsystem {
  private static Conveyor mInstance;

  public static Conveyor getInstance(){
    if(mInstance == null){
      mInstance = new Conveyor();
    }
    return mInstance;
  }
  
  // motors
  CANSparkMax mIntakeA = new CANSparkMax(Constants.kIntakeA, MotorType.kBrushless);
  CANSparkMax mIntakeB = new CANSparkMax(Constants.kIntakeB, MotorType.kBrushless);
  CANSparkMax mIntakeC = new CANSparkMax(Constants.kIntakeC, MotorType.kBrushless);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
