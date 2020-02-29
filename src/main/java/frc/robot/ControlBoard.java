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
        if(Math.abs(xbox.getY(Hand.kLeft)) > 0.10){
            return -xbox.getY(Hand.kLeft);
        }else{
            return 0;
        }
        
    }

    public double getTurn(){
        if(Math.abs(xbox.getX(Hand.kRight)) > 0.10){
            return xbox.getX(Hand.kRight);
        }else{
            return 0;
        }
    }
    public boolean getEjectIntake(){
        return xbox.getBackButton();
    }
    public boolean getReleaseWinch(){
        return xbox.getPOV() == 0;
    }
    public boolean getEnageWinch(){
        return xbox.getPOV() == 180;
    }
    LatchedBoolean reverseEdge = new LatchedBoolean();
    public boolean getReverse(){
        return reverseEdge.update(xbox.getXButton());
    }
    public boolean getReadyShooter(){
        return xbox.getBumper(Hand.kRight);
    }
    LatchedBoolean driverIntake = new LatchedBoolean();
    public boolean getIntakeButton(){
        return driverIntake.update(xbox.getBumper(Hand.kLeft));
    }
    LatchedBoolean spinEdge = new LatchedBoolean();
    public boolean getSpinFlywheel(){
        return spinEdge.update(xbox.getYButton());
    }
    // LatchedBoolean gateEdge = new LatchedBoolean();
    public boolean getManualGate(){
        return xbox.getAButton();
    }
    LatchedBoolean shiftEdge = new LatchedBoolean();
    public boolean getShift(){
        return shiftEdge.update(xbox.getTriggerAxis(Hand.kRight) > 0.3);
    }
    public boolean getShootButton(){
        return xbox.getBButton();
    }
    public boolean getSlowMode(){
        return xbox.getTriggerAxis(Hand.kRight) > 0.3;
    }

    //OPERATOR CONTROLS
    public boolean getOperatorReadyShooter(){
        return operator.getBumper(Hand.kRight);
    }
    public boolean getOperatorShoot(){
        return operator.getBButton();
    }
    LatchedBoolean disableEdge = new LatchedBoolean();
    public boolean getOperatorToggleClimbPiston(){
        return disableEdge.update(operator.getXButton());
    }
    public double getOperatorTurretControl(){//mapped to 35%
        return -operator.getX(Hand.kRight)*0.35;
    }
    LatchedBoolean overrideEdge = new LatchedBoolean();
    public boolean getOperatorToggleTurretOverride(){
        return overrideEdge.update(operator.getYButton());
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
    LatchedBoolean climbEdge = new LatchedBoolean();
    public boolean getClimb(){
        return climbEdge.update(operator.getStartButton());
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
