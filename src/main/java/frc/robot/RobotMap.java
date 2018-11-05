package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.IdGroup;

public class RobotMap
{
    public static int DRIVER_JOYSTICK_ID = 0;

    public static int CAN_TIMEOUT_MSEC = 10;    // Only use this during initialization; use zero for periodic calls

    public static int DRIVE_MOTOR_LEFT_A_ID  = 11;
    public static int DRIVE_MOTOR_LEFT_B_ID  = 12;
    public static int DRIVE_MOTOR_RIGHT_A_ID = 13;
    public static int DRIVE_MOTOR_RIGHT_B_ID = 14;

    final public static int[] DRIVE_MOTOR_LEFT_ID_GROUP = 
    {
        DRIVE_MOTOR_LEFT_A_ID,
        DRIVE_MOTOR_LEFT_B_ID
    };
    final public static int[] DRIVE_MOTOR_RIGHT_ID_GROUP = 
    {
        DRIVE_MOTOR_RIGHT_A_ID,
        DRIVE_MOTOR_RIGHT_B_ID
    };    


    final public static FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
};