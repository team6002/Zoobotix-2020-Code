
package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.Drive;

//file to hold constants used around the robot.

public class Constants{
    //drive values (generated from frc characterization tool)
    public static final double ksVolts = 0.101;//0.0982
    public static final double kvVoltSecondsPerMeter = 3.94;//3.87;
    public static final double kaVoltSecondsSquaredPerMeter = 0.448;//0.39;
    //Ramsete controller pid for left and right controllers
    public static final double kDrivePVel = 1.76;//14.7;
    public static final double kDriveDVel = 0;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //hardware facts
    public static final double kTrackWidthInches = 24.528;
    public static final double kTrackWidthMeters = 0.628;
    public static final double kCountsPerRevolution = 42; //for the neo's internal encoder
    
    //wheel stats
    public static final double kWheelRadiusInches = 3;
    public static final double kWheelCircumferenceInches = 2 * Math.PI * kWheelRadiusInches;
    public static final double kTicksPerInch = 360/kWheelCircumferenceInches;
    

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kLowGearRatio = 15;
    public static final double kHighGearRatio = 7.29;
    //drive ports
    public final static int kDriveLeftMaster = 15;//1;
    public final static int kDriveLeftSlave = 14;//16;
    public final static int kDriveRightMaster = 16;//14;
    public final static int kDriveRightSlave = 1;//15;

    public final static int kShifter = 3;

    public final static double kLooperDt = 0.01;

    //Drive pid constants
    public final static double kDt = 0.02;
    public final static double kDriveP = 0;
    public final static double kDriveI = 0;
    public final static double kDriveD = 0;
    public final static TrapezoidProfile.Constraints kDriveConstraints = new TrapezoidProfile.Constraints(Constants.kDriveMaxVel, Constants.kDriveMaxAccel);

    public final static double kDriveMaxVel = 2;
    public final static double kDriveMaxAccel = 1;

    public final static double straightkP = 0.02;

    //Drive neo pid constants
    public final static double kNeoDriveP = 0.0;//5e-5;
    public final static double kNeoDriveI = 0;
    public final static double kNeoDriveD = 0;
    public final static double kNeoDriveIz = 0;
    public final static double kNeoDriveF = 0;
    public final static double kMinOutput = -1;
    public final static double kMaxOutput = 1;

    
    //shooter ports 
    public final static int kShooterMaster = 2;
    public final static int kShooterSlave = 3;

    //shooter pid constants
    public final static double kShooterP = 0.00068;
    public final static double kShooterI = 0;
    public final static double kShooterD = 0.00000750;
    public final static double kShooterIz = 0;
    public final static double kShooterF = 0.000180;

    public final static double kShooterMaxVel = 5000;
    public final static double kShooterMaxAccel = 2500;

    //turret ports
    public final static int kTurret = 5;

    //turret pid constants
    public final static int kTurretP = 0;
    public final static int kTurretI = 0;
    public final static int kTurretD = 0;
    public final static int kTurretIz = 0;
    public final static double kTurretF = 0;
    
    //Intake ports
    public final static int kTopIntake = 10;
    public final static int kBotIntake = 4;
    public final static int kGateIntake = 11;
    public final static int kDeployIntake = 13;

    

    //Indexer/Intake sensors
    public final static int kCellSensor = 0;
    public final static int kEmptySensor = 1;
    public final static int kFullSensor = 2;

    //Climber Ports
    public final static int kWinchMaster = 6;
    public final static int kBalance = 9;
    //solenoids
    public final static int kWinchRatchet = 0;
    public final static int kClimberDeploy = 2;
    public final static int kIntakeOut = 1;
    // public final static int kIntakeDown = 5;

    //climber pid constants
    public final static double kClimberP = 0.05;
    public final static double kClimberI = 0.00000001;
    public final static double kClimberD = 0;
    public final static double kClimberIz = 0;
    public final static double kClimberF = 0;

    public final static double kClimberMaxAccel = 5;
    public final static double kClimberMaxVel = 5;

    // control panel port
    public final static int kControlPanel = 11;
    public final static int kControlPanelSolenoid = 7;

    //Ramsete Controller configs and stuff
    
      

    

}