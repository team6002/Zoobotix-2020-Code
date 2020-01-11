
package frc.robot;

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

    public final static double kDriveMaxVel = 5;
    public final static double kDriveMaxAccel = 5;

    //wheel stats
    public final static double kWheelCircumference = Math.PI * 6;
    public final static double kTicksPerInch = 360/kWheelCircumference;

}