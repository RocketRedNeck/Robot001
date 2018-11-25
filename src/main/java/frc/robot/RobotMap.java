package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class RobotMap
{
    final public static double ROBOT_PERIOD_SEC = 0.02;
    final public static double ROBOT_RATE_HZ    = 1.0 / ROBOT_PERIOD_SEC;

    final public static int DRIVER_JOYSTICK_ID = 0;
    final public static int OPERATOR_JOYSTICK_ID = 1;

    final public static int NUM_CONTROLLER_FEEDBACK_LOOPS = 1;  // CTRE TalonSRX has 2 but the Auxilliary is not available yet
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


    final public static double DRIVE_SPEED_LIMIT_FACTOR = 0.4;
    final public static double DRIVE_TURN_LIMIT_FACTOR  = 0.2;
    final public static double DRIVE_TANK_LIMIT_FACTOR  = 0.6;

    final public static FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
    final public static int DRIVE_MOTOR_FEEDBACK_TICK_PER_REV = 8192;
    final public static int DRIVE_QUADRATURE_STATUS_FRAME_PERIOD_MSEC = 100;

    // In this specific robot the left motor and sensor is inverted so positive
    // input makes the robot move "forward"
    final public static boolean DRIVE_LEFT_MOTOR_INVERSION = true;
    final public static boolean DRIVE_RIGHT_MOTOR_INVERSION = false;
    
    final public static boolean DRIVE_LEFT_SENSOR_PHASE  = true;
    final public static boolean DRIVE_RIGHT_SENSOR_PHASE = false;

    final public static double DRIVE_TICKS_PER_REV_PER_100MS_PER_MIN = RobotMap.DRIVE_MOTOR_FEEDBACK_TICK_PER_REV/600.0;
    public static double driveTicksToRpm(double ticksPer100Ms)
    {
        return ticksPer100Ms / DRIVE_TICKS_PER_REV_PER_100MS_PER_MIN;
    }
    public static double driveRpmToTicks(double rpm)
    {
        return rpm * DRIVE_TICKS_PER_REV_PER_100MS_PER_MIN;
    }    
};