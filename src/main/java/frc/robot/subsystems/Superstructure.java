/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.LatchedBoolean;
import frc.robot.loops.*;
import frc.robot.subsystems.Turret.Hint;

/**
 * Everything above the drivetrain
 * Makes sure that everything works together.
 */
public class Superstructure extends Subsystem {
  private Intake mIntake = Intake.getInstance();
  private Shooter mShooter = Shooter.getInstance();
  private Turret mTurret = Turret.getInstance();
  private Climber mClimber = Climber.getInstance();
  private ControlPanel mControlPanel = ControlPanel.getInstance();

  public enum WantedState {
    IDLE,
    SHOOT,
    INTAKE,
    DEPLOY_CONTROL_PANEL,
    WANT_CLIMB,
  }

  private WantedState mWantedState = WantedState.IDLE;

  public enum SystemState {
    IDLE,
    INTAKING,
    DEPLOYED_CONTROL_PANEL,
    READY_TO_SHOOT,
    WAITING_FOR_FLYWHEEL,
    SHOOTING,
    CLIMB,
  }
  private SystemState mSystemState = SystemState.IDLE;


  private static Superstructure mInstance = null; 
    public synchronized static Superstructure getInstance(){
        if (mInstance == null) {
        mInstance = new Superstructure();
        }
        return mInstance;
    }

    private boolean mStateChanged;
    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
              mStateChanged = true;
            }
        }
    
        @Override
        public void onLoop(double timestamp) {
          SystemState newState = mSystemState;

          synchronized (Superstructure.this) {
            switch(mSystemState){
              case IDLE:
                newState = handleIdle();
                break;
              case INTAKING:
                newState = handleIntaking();
                break;
              case DEPLOYED_CONTROL_PANEL:
                newState = handleDeployedControlPanel();
                break;
              case WAITING_FOR_FLYWHEEL:
                newState = handleWaitingForFlywheel();
                break;
              case READY_TO_SHOOT:
                newState = handleReadyToShoot();
                break;
              case CLIMB: 
                newState = handleClimb();
                break;
              default:
                newState = SystemState.IDLE;
              break;
          }

          if(newState != mSystemState){
            System.out.println("Superstructure state " + mSystemState + " to " + newState + " TimeStamp: " + Timer.getFPGATimestamp());
            mSystemState = newState;
            mStateChanged = true;
          }else{
            mStateChanged = false;
          }
        }
      }
    
      @Override
      public void onStop(double timestamp) {
          stop();
    
      }
  };
  public void registerEnabledLoops(ILooper in) {
    in.register(mLoop);
  }

  private SystemState defaultStateTransfer(){
    switch(mWantedState){
      case SHOOT:
        return SystemState.SHOOTING;
      case DEPLOY_CONTROL_PANEL:
        return SystemState.DEPLOYED_CONTROL_PANEL;
      case INTAKE:
        return SystemState.INTAKING;
      case IDLE:
        return SystemState.IDLE;
      default:
        return SystemState.IDLE;
    }
  }

  private SystemState handleIdle(){
    return defaultStateTransfer();
  }

  private SystemState handleIntaking(){
    mIntake.startIntaking();

    switch(mWantedState){
      case INTAKE:
        return SystemState.INTAKING;
      case IDLE:
        mIntake.setOff();
        return SystemState.IDLE;
      case DEPLOY_CONTROL_PANEL:
        mIntake.setOff();
        return SystemState.DEPLOYED_CONTROL_PANEL;
      case SHOOT:
        if(mShooter.isReady){
          mIntake.setOff();
          return SystemState.READY_TO_SHOOT;
        }else{
          mIntake.setOff();
          return SystemState.WAITING_FOR_FLYWHEEL;
        }
      default:
        return SystemState.INTAKING;
    }
  }

  private SystemState handleDeployedControlPanel(){
    mControlPanel.deployControlPanel();

    switch(mWantedState){
      case INTAKE:
        mControlPanel.stowControlPanel();
        return SystemState.INTAKING;
      case DEPLOY_CONTROL_PANEL:
        return SystemState.DEPLOYED_CONTROL_PANEL;
      case SHOOT:
        mControlPanel.stowControlPanel();
        if(mShooter.isReady){
          return SystemState.READY_TO_SHOOT;
        }else{
          return SystemState.WAITING_FOR_FLYWHEEL;
        }
      case IDLE:
        mControlPanel.stowControlPanel();
        return SystemState.IDLE;
      default:
        return SystemState.DEPLOYED_CONTROL_PANEL;
    }
  }

  private SystemState handleWaitingForFlywheel(){
    //instead of hard coding value, need to get value from look up distance-rpm table
    mShooter.setVelocity(2500);

    switch(mWantedState){
      case INTAKE:
        return SystemState.INTAKING;
      case DEPLOY_CONTROL_PANEL:
        return SystemState.DEPLOYED_CONTROL_PANEL;
      case SHOOT:
        if(mShooter.isReady){
          return SystemState.READY_TO_SHOOT;
        }else{
          return SystemState.WAITING_FOR_FLYWHEEL;
        }
      case IDLE:
        return SystemState.IDLE;
      default:
        if(mShooter.isReady){
          return SystemState.READY_TO_SHOOT;
        }else{
          return SystemState.WAITING_FOR_FLYWHEEL;
        }
    }
  }

  LatchedBoolean shotEdge = new LatchedBoolean();
  private SystemState handleReadyToShoot(){
    //start targetting
    if(!mTurret.getWantTarget()) mTurret.setTurretState(true);

    if(wantShoot){
      if(mShooter.isReady && mTurret.onTarget){
        //use the indexer to push a ball up and fire
        mIntake.startFeeding();
        if(shotEdge.update(mShooter.shotBall)) mIntake.decreaseCellCount();
      }else{
        mIntake.setOff();
        //wait on targeting or spin up.
      }
    }else{
      mIntake.setOff();
      //keep the flywheel spinning
    }

    
    
    if(mIntake.isEmpty()){//no more balls to shoot
      mWantedState = WantedState.IDLE;
    }

    switch(mWantedState){
      case INTAKE:
        return SystemState.INTAKING;
      case SHOOT:
        return SystemState.SHOOTING;
      case IDLE:
        return SystemState.IDLE;
      default:
        return SystemState.SHOOTING;
    }
  }

  private SystemState handleClimb(){
    if(mStateChanged){
      mClimber.releaseWinch();
      //TODO code the deployment of climber :)
    }

    switch(mWantedState){
      case WANT_CLIMB:
        return SystemState.CLIMB;
      case INTAKE:
        return SystemState.INTAKING;
      case SHOOT:
        return SystemState.SHOOTING;
      default:
        return SystemState.CLIMB;
    }
  }

  private boolean wantShoot = false;
  public void shoot(boolean yes){
    if(yes) wantShoot = true;
    else wantShoot = false;
  }

  public void wantRotationControl(){
    mControlPanel.doRotation();
  }

  public void setTurretHint(Hint hint){
    mTurret.setHint(hint);
  }

  public void setWantedState(WantedState wantedState){
    mWantedState = wantedState;
  }

  public SystemState getSystemState(){
    return mSystemState;
  }

  public synchronized void stop(){
    mIntake.stop();
    mShooter.stop();
    mTurret.stop();
  }

  public void outputToSmartDashboard(){
    SmartDashboard.putString("Superstructure State", getSubsystem().toString());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
