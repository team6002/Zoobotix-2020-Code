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
    private XboxController operator = new XboxController(1);

    private LatchedBoolean aEdge = new LatchedBoolean();
    private LatchedBoolean bEdge = new LatchedBoolean();
    private LatchedBoolean xEdge = new LatchedBoolean();
    private LatchedBoolean yEdge = new LatchedBoolean();

    public double getThrottle(){
        return xbox.getY(Hand.kLeft);
    }

    public double getTurn(){
        return xbox.getX(Hand.kRight);
    }

    public boolean getA(){
        return xbox.getAButton();
    }
    public boolean getB(){
        return xbox.getBButton();
    }
    public boolean getX(){
        return xbox.getXButtonPressed();
    }
    public boolean getY(){
        return xbox.getYButtonPressed();
    }
    public boolean getReadyShooter(){
        return xbox.getBumper(Hand.kRight);
    }
    public boolean getIntakeButton(){
        return xbox.getBumper(Hand.kLeft);
    }
    public boolean getShiftUp(){
        return xbox.getTriggerAxis(Hand.kRight) > 0.3;
    }
    public boolean getShiftDown(){
        return xbox.getTriggerAxis(Hand.kLeft) > 0.3;
    }
    public boolean getShootButton(){
        return xbox.getBButton();
    }

    //operator controls
    public boolean getOperatorReadyShooter(){
        return operator.getBumper(Hand.kRight);
    }
    public boolean getOperatorShoot(){
        return operator.getBButton();
    }
    public boolean getOperatorIntake(){
        return operator.getBumper(Hand.kLeft);
    }
    public boolean getOperatorIntakeOff(){
        return operator.getTriggerAxis(Hand.kLeft) > 0.3;
    }
    public boolean getOperatorIntakeReverse(){
        return operator.getXButton();
    }
    LatchedBoolean controlPanelEdge = new LatchedBoolean();
    public boolean getOperatorControlPanel(){
        return controlPanelEdge.update(operator.getYButton());
    }
    public boolean getClimb(){
        return operator.getStartButton();
    }
    public double getClimbStick(){
        return -operator.getY(Hand.kLeft);
    }
    public double getBalanceStick(){
        return operator.getX(Hand.kRight);
    }
    public boolean getHintLeft(){
        return operator.getPOV() == 270;
    }
    public boolean getHintRight(){
        return operator.getPOV() == 90;
    }
    public boolean getControlPanelLeft(){
        return operator.getPOV() == 270;
    }
    public boolean getControlPanelRight(){
        return operator.getPOV() == 90;
    }
    public boolean getControlPanelSpin(){
        return operator.getPOV() == 0;
    }
}
