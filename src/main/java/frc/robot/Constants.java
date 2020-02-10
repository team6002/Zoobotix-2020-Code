
package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

//file to hold constants used around the robot.

public class Constants{
    //drive ports
    public final static int kDriveLeftMaster = 1;//1;
    public final static int kDriveLeftSlave = 2;//16;
    public final static int kDriveRightMaster = 3;//14;
    public final static int kDriveRightSlave = 4;//15;

    public final static int kShifter = 0;

    public final static double kLooperDt = 0.01;

    //Drive pid constants
    public final static double kDt = 0.02;
    public final static double kDriveP = 0;
    public final static double kDriveI = 0;
    public final static double kDriveD = 0;
    public final static TrapezoidProfile.Constraints kDriveConstraints = new TrapezoidProfile.Constraints(Constants.kDriveMaxVel, Constants.kDriveMaxAccel);

    public final static double kDriveMaxVel = 2;
    public final static double kDriveMaxAccel = 1;

    //Drive neo pid constants
    public final static double kNeoDriveP = 5e-5;
    public final static double kNeoDriveI = 0;
    public final static double kNeoDriveD = 0;
    public final static double kNeoDriveIz = 0;
    public final static double kNeoDriveF = 0;
    public final static double kMinOutput = -1;
    public final static double kMaxOutput = 1;

    
    //shooter ports 
    public final static int kShooterMaster = 14;//1
    public final static int kShooterSlave = 15;//15

    //shooter pid constants
    public final static double kShooterP = 0.000023;
    public final static double kShooterI = 0;//0.0000003;
    public final static double kShooterD = 0;
    public final static double kShooterIz = 0;
    public final static double kShooterF = 0.000194;

    public final static double kShooterMaxVel = 5000;
    public final static double kShooterMaxAccel = 2500;

    //turret ports
    public final static int kTurret = 13;

    //turret pid constants
    public final static int kTurretP = 0;
    public final static int kTurretI = 0;
    public final static int kTurretD = 0;
    public final static int kTurretIz = 0;
    public final static double kTurretF = 0;


    //indexer port
    //TODO decide if we need this
    public final static int kIndexer = 0;


    //indexer pid constants
    public final static double kIndexerP = 0;
    public final static double kIndexerI = 0;
    public final static double kIndexerD = 0;
    public final static double kIndexerIz = 0;
    public final static double kIndexerF = 0;

    public final static double kIndexerMaxVel = 50;
    public final static double kIndexerMaxAccel = 50;

    
    //Intake ports
    public final static int kTopIntake = 5;
    public final static int kBotIntake = 6;
    public final static int kGateIntake = 7;
    public final static int kDeployIntake = 8;

    public final static int kIntakeOut = 1;
    public final static int kIntakeDown = 2;

    public final static int kCellSensor = 0;

    //Climber Ports
    public final static int kWinchMaster = 9;
    public final static int kWinchSlave = 10;
    public final static int kWinchRatchet = 3;
    public final static int kBalance = 12;

    // control panel port
    public final static int kControlPanel = 11;
    public final static int kControlPanelSolenoid = 4;

    //wheel stats
    public final static double kWheelCircumference = Math.PI * 6;
    public final static double kTicksPerInch = 360/kWheelCircumference;

}