package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class RobotMap
{
    final public static int DRIVER_JOYSTICK_ID = 0;

    final public static int CAN_TIMEOUT_MSEC = 10;    // Only use this during initialization; use zero for periodic calls

    final public static int DRIVE_MOTOR_LEFT_A_ID  = 1;
    final public static int DRIVE_MOTOR_LEFT_B_ID  = 4;
    final public static int DRIVE_MOTOR_RIGHT_A_ID = 2;
    final public static int DRIVE_MOTOR_RIGHT_B_ID = 3;

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
    final public static int DRIVE_MOTOR_FEEDBACK_TICK_PER_REV = 8192;
};