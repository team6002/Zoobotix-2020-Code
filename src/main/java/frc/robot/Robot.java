/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.Looper;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SystemState;
import frc.robot.subsystems.Superstructure.WantedState;

import frc.robot.subsystems.*;
import frc.robot.auto.AutoContainer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Superstructure mSuperstructure = Superstructure.getInstance();
  public static Drive mDrive = Drive.getInstance();
  public static Intake mIntake = Intake.getInstance();
  public static Shooter mShooter = Shooter.getInstance();
  public static Turret mTurret = Turret.getInstance();
  // public static ControlPanel mControlPanel = ControlPanel.getInstance();
  public static ControlBoard mControlBoard = ControlBoard.getInstance();
  public static Climber mClimber = Climber.getInstance();

  Compressor c;
  

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable mTable = inst.getTable("SmartDashboard");

  AutoContainer mAutoContainer = new AutoContainer();

  private Looper mEnabledLooper = new Looper();
  
  
  String gameData;


  Command mAutonomousCommand;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    mSuperstructure.registerEnabledLoops(mEnabledLooper);
    // mDrive.registerEnabledLoops(mEnabledLooper);
    mIntake.registerEnabledLoops(mEnabledLooper);
    mShooter.registerEnabledLoops(mEnabledLooper);
    mTurret.registerEnabledLoops(mEnabledLooper);

    c = new Compressor(0);
    c.setClosedLoopControl(true);
    
    //TODO remove shooter testing function
    SmartDashboard.putNumber("Shooter RPM", 2500);

    mDrive.outputToSmartDashboard();
    updateTelemetry();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //for running autonomous container
    CommandScheduler.getInstance().run();

    gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0){
      SmartDashboard.putString("Color For Position", gameData);
    }else{
      SmartDashboard.putString("Color For Position", "N/A");
    }
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    mEnabledLooper.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    mDrive.resetOdometry();
    mAutonomousCommand = mAutoContainer.getAutonomousCommand();


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (mAutonomousCommand != null) {
      mAutonomousCommand.schedule();
    }

    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    mDrive.outputToSmartDashboard();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }

    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  //software toggles
  boolean intakeToggle = false;
  boolean isReverseControl = false;
  @Override
  public void teleopPeriodic() {
    if(mControlBoard.getClimb()){
      mSuperstructure.setWantedState(WantedState.WANT_CLIMB);
    }

    SmartDashboard.putNumber("throttle", mControlBoard.getThrottle());
    double throttle = mControlBoard.getThrottle();
    double turn = mControlBoard.getTurn();

    if(mControlBoard.getShift()){
      if(mDrive.isHighGear()){
        mDrive.shift(false);
      }else{
        mDrive.shift(true);
      }
    }
    
    if(mControlBoard.getSlowMode()){
      throttle = throttle * 0.65;
      turn = turn * 0.65;
    }

    if(mControlBoard.getReverse()){
      if (isReverseControl){
        isReverseControl = false;
      }
      else{
        isReverseControl = true;
      }
    }

    if(isReverseControl == true){
      throttle = throttle  * -1;
      turn = turn ;
    }

    mDrive.arcadeDrive(throttle, turn);
    
    if(mControlBoard.getIntakeButton()){
      if(mSuperstructure.getSystemState() != SystemState.INTAKING){
        mSuperstructure.setWantedState(WantedState.INTAKE);
      }else{
        mSuperstructure.setWantedState(WantedState.IDLE);
      }
    }
    
    if(mControlBoard.getReadyShooter()){
      mSuperstructure.setWantedState(WantedState.SHOOT);
    }

    if(mControlBoard.getOperatorToggleTurretOverride()){
      mSuperstructure.toggleTurretOverride();
    }


    if(mSuperstructure.getSystemState() == SystemState.INTAKING){
      if(mControlBoard.getManualGate()){
        mIntake.gateOn();
      }else{
        mIntake.gateOff();
      }
    }

    if(mSuperstructure.getSystemState() == SystemState.READY_TO_SHOOT){
      if(mControlBoard.getShootButton()){
        mSuperstructure.shoot(true);
      }else{
        mSuperstructure.shoot(false);
      }

      if(mControlBoard.getSpinFlywheel()){
        if(mSuperstructure.getWantSpinFlywheel()){
          mSuperstructure.stopFlywheel();
        }else{
          mSuperstructure.spinFlywheel();
        }
      }

      if(mSuperstructure.getOperatorTurretOverride()){
        mTurret.setOpenLoop(mControlBoard.getOperatorTurretControl());
      }
    }else {
      mSuperstructure.stopFlywheel();
      mSuperstructure.shoot(false);
    }

    

    

    if(mSuperstructure.getSystemState() == SystemState.CLIMB){
      mClimber.setWinch(mControlBoard.getClimbStick());
      // mClimber.setBalance(mControlBoard.getBalanceStick());
    }
    

      //   if(mControlBoard.getHintLeft()){
  //     mSuperstructure.setTurretHint(Hint.LEFT);
  //   }
  //   if(mControlBoard.getHintRight()){
  //     mSuperstructure.setTurretHint(Hint.RIGHT);
  //   }
  //   if(mControlBoard.getOperatorControlPanel()){
  //     mSuperstructure.setWantedState(WantedState.DEPLOY_CONTROL_PANEL);
  //   }
    
  //   if(mSuperstructure.getSystemState() == SystemState.DEPLOYED_CONTROL_PANEL){
  //     if(mControlBoard.getControlPanelSpin()){
  //       mSuperstructure.wantRotationControl();
  //     }else if(mControlBoard.getControlPanelLeft()){
  //       mControlPanel.setOpenLoop(-0.1);
  //     }else if(mControlBoard.getControlPanelRight()){
  //       mControlPanel.setOpenLoop(0.1);
  //     }
  //   }
    
    updateTelemetry();
  }

  public void updateTelemetry(){
    SmartDashboard.putBoolean( "reverse button", isReverseControl);
    mSuperstructure.outputToSmartDashboard();
    mDrive.outputToSmartDashboard();
    mClimber.outputToSmartDashboard();
    mShooter.outputToSmartDashboard();
    mTurret.outputToSmartdashboard();
    mIntake.outputToSmartDashboard();

    boolean enabled = c.enabled();
    boolean pressureSwitch = c.getPressureSwitchValue();
    double current = c.getCompressorCurrent();
    SmartDashboard.putBoolean("Compressor", enabled);
    SmartDashboard.putBoolean("Pressure Switch", pressureSwitch);
    SmartDashboard.putNumber("Compressor Current", current);
    SmartDashboard.putNumber("estimated distance", distanceFromTarget(mTable.getEntry("width").getDouble(-1)));
    // SmartDashboard.putNumber("top", mTable.getEntry("h").getDouble(-1));
  }
  
  public double distanceFromTarget(double x){
    if(x == -1) return -1;
    return 82.6 - 2.44*x + 0.03*Math.pow(x, 2) -1.36e-4*Math.pow(x, 3);
  }

  public double calculateTop(double cY, double h){
    return cY + h/2;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
