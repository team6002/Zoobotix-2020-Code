/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.ControlBoard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.SystemState;
import frc.robot.subsystems.Superstructure.WantedState;
import frc.robot.subsystems.Turret.Hint;
import frc.robot.loops.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static Superstructure mSuperstructure = Superstructure.getInstance();
  public static Drive mDrive = Drive.getInstance();
  public static Shooter mShooter = Shooter.getInstance();
  public static Turret mTurret = Turret.getInstance();
  public static ControlPanel mControlPanel = ControlPanel.getInstance();
  public static ControlBoard mControlBoard = ControlBoard.getInstance();
  public static Climber mClimber = Climber.getInstance();

  private Looper mEnabledLooper = new Looper();

  String gameData;


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    mSuperstructure.registerEnabledLoops(mEnabledLooper);
    mDrive.registerEnabledLoops(mEnabledLooper);
    mShooter.registerEnabledLoops(mEnabledLooper);
    mTurret.registerEnabledLoops(mEnabledLooper);

    //initialize networktables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();


    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
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
    Scheduler.getInstance().run();
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
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }

    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    mEnabledLooper.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();


    mDrive.arcadeDrive(mControlBoard.getThrottle(), mControlBoard.getTurn());
    
    if(mControlBoard.getIntakeButton()){
      mSuperstructure.setWantedState(WantedState.INTAKE);
    }
    if(mControlBoard.getReadyShooter()){
      mSuperstructure.setWantedState(WantedState.SHOOT);
    }

    if(mControlBoard.getHintLeft()){
      mSuperstructure.setTurretHint(Hint.LEFT);
    }
    if(mControlBoard.getHintRight()){
      mSuperstructure.setTurretHint(Hint.RIGHT);
    }
    if(mControlBoard.getOperatorControlPanel()){
      mSuperstructure.setWantedState(WantedState.DEPLOY_CONTROL_PANEL);
    }
    
    if(mSuperstructure.getSystemState() == SystemState.DEPLOYED_CONTROL_PANEL){
      if(mControlBoard.getControlPanelSpin()){
        mSuperstructure.wantRotationControl();
      }else if(mControlBoard.getControlPanelLeft()){
        mControlPanel.setOpenLoop(-0.1);
      }else if(mControlBoard.getControlPanelRight()){
        mControlPanel.setOpenLoop(0.1);
      }
    }

    if(mSuperstructure.getSystemState() == SystemState.READY_TO_SHOOT){
      if(mControlBoard.getShootButton()){
        mSuperstructure.shoot(true);
      }else{
        mSuperstructure.shoot(false);
      }
    }else {
      mSuperstructure.shoot(false);
    }

    if(mControlBoard.getClimb()){
      mSuperstructure.setWantedState(WantedState.WANT_CLIMB);
    }

    if(mSuperstructure.getSystemState() == SystemState.CLIMB){
      mClimber.setWinch(mControlBoard.getClimbStick());
      mClimber.setBalance(mControlBoard.getBalanceStick());
    }
    


    mShooter.outputToSmartDashboard();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
