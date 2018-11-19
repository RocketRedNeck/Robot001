package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI
{
    public static Joystick driverControl = new Joystick(RobotMap.DRIVER_JOYSTICK_ID);
    public static Joystick opertatorControl = new Joystick(RobotMap.OPERATOR_JOYSTICK_ID);

    final public static int DRIVE_SPEED_AXIS            = PS4Constants.LEFT_STICK_Y.getValue();
    final public static int DRIVE_TURN_AXIS             = PS4Constants.RIGHT_STICK_X.getValue();
    final public static int DRIVE_RIGHT_SPEED_AXIS      = PS4Constants.RIGHT_STICK_Y.getValue();
    final public static int TEST_VELOCITY_MODE_BUTTON   = PS4Constants.CIRCLE.getValue();

    public static double speed()
    {
        return driverControl.getRawAxis(DRIVE_SPEED_AXIS);
    }
    public static double leftSpeed()
    {
        return speed();
    }
    public static double rightSpeed()
    {
        return driverControl.getRawAxis(DRIVE_RIGHT_SPEED_AXIS);
    }

    public static double turn()
    {
        return -driverControl.getRawAxis(DRIVE_TURN_AXIS);
    }

    public static boolean testVelocityMode()
    {
        return driverControl.getRawButton(TEST_VELOCITY_MODE_BUTTON);
    }

}