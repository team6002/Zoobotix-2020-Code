
package frc.robot;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

//file to hold constants used around the robot.

public class Constants{
    //drive ports
    public final static int kDriveLeftMaster = 1;
    public final static int kDriveLeftSlave = 16;
    public final static int kDriveRightMaster = 14;
    public final static int kDriveRightSlave = 15;

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
    public final static int kShooterMaster = 0;
    public final static int kShooterSlave = 0;

    

    //shooter pid constants
    public final static double kShooterP = 0.2;
    public final static double kShooterI = 0;
    public final static double kShooterD = 0;
    public final static double kShooterIz = 0;
    public final static double kShooterF = 0;

    public final static double kShooterMaxVel = 100;
    public final static double kShooterMaxAccel = 10;

    //indexer port
    public final static int kIndexer = 0;

    //indexer pid constants
    public final static double kIndexerP = 0.2;
    public final static double kIndexerI = 0;
    public final static double kIndexerD = 0;
    public final static double kIndexerIz = 0;
    public final static double kIndexerF = 0;

    public final static double kIndexerMaxVel = 5;
    public final static double kIndexerMaxAccel = 5;

    //wheel stats
    public final static double kWheelCircumference = Math.PI * 6;
    public final static double kTicksPerInch = 360/kWheelCircumference;

}