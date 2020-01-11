/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.lib.util.LatchedBoolean;
/**
 * Add your docs here.
 */
public class ControlBoard {
    private static ControlBoard mInstance = null;

    public synchronized static ControlBoard getInstance(){
        if(mInstance == null){
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private XboxController xbox = new XboxController(0);

    public double getThrottle(){
        return xbox.getY(Hand.kLeft);
    }

    public double getTurn(){
        return xbox.getX(Hand.kRight);
    }

    
}
