/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivesubsystem;

import frc.robot.PS4Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private WPI_TalonSRX[] leftMotors;
    private WPI_TalonSRX[] rightMotors;

    private double[] leftCurrent_amp;
    private double[] rightCurrent_amp;

    private double leftPosition_ticks;
    private double rightPosition_ticks;

    private double leftVelocity_ticksPsec;
    private double rightVelocity_ticksPsec;


    private final FeedbackDevice FEEDBACK_DEVICE;
    private final int CAN_TIMEOUT_MSEC;

    private DifferentialDrive drive;

    private enum DriveStyles 
    {
        WPI_ARCADE, 
        WPI_CURVATURE, 
        WPI_TANK, 
        MODIFIED_ARCADE,
    };

    private SendableChooser<DriveStyles> driveStyleChooser;

    /***
     * Constructor
     */
    public DriveSubsystem(int[] leftIds, 
                          int[] rightIds, 
                          FeedbackDevice aFeedbackDevice,
                          int aTimeout_msec) 
    {
        driveStyleChooser = new SendableChooser<DriveStyles>(); // put this on the dashboard at initialization
        driveStyleChooser.addDefault("WPI Arcade",      DriveStyles.WPI_ARCADE);
        driveStyleChooser.addObject( "WPI Curvature",   DriveStyles.WPI_CURVATURE);
        driveStyleChooser.addObject( "WPI Tank",        DriveStyles.WPI_TANK);
        driveStyleChooser.addObject( "Modified Arcade", DriveStyles.MODIFIED_ARCADE);

        // Configure motors at initialiation
        leftMotors = new WPI_TalonSRX[leftIds.length];
        rightMotors = new WPI_TalonSRX[rightIds.length];

        leftCurrent_amp = new double[leftMotors.length];
        rightCurrent_amp = new double[rightMotors.length];

        for (int i = 0; i < leftMotors.length; ++i) 
        {
            leftMotors[i] = new WPI_TalonSRX(leftIds[i]);
            leftMotors[i].setName(getName(),"Left_" + Integer.toString(i));            
        }
        for (int i = 0; i < rightMotors.length; ++i) 
        {
            rightMotors[i] = new WPI_TalonSRX(rightIds[i]);
            rightMotors[i].setName(getName(),"Right_" + Integer.toString(i));            
        }

        FEEDBACK_DEVICE = aFeedbackDevice;
        CAN_TIMEOUT_MSEC = aTimeout_msec;

        // All other motors will be followers so we only need to initialize the drive
        // with the first motor in each group
        // NOTE: We don't use the SpeedControllerGroup because the behavior requires
        // additional CAN traffic that is not needed with TalonSRX controllers.
        drive = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    }

    /**
     * initializeMotor - set all of the motor configuration states to a known value
     * This is important when we are not sure if the motor is in a factory state
     * @param aMotor
     * 
     * TODO: Move this to a separate package?
     */
    private void initializeMotor(WPI_TalonSRX aMotor)
    {
        // TODO: Check ErrorCode?

        aMotor.stopMotor();

        // Group all of the slot configurations together for a loop
        for (int slotIdx = 0; slotIdx < 2; ++slotIdx)   // TODO: Source MAGIC NUMBER 2, what is the actual number of slots in the firmware?
        {
            aMotor.config_IntegralZone(slotIdx, 0, CAN_TIMEOUT_MSEC);
            aMotor.config_kD(slotIdx, 0.0, CAN_TIMEOUT_MSEC);
            aMotor.config_kF(slotIdx, 0.0, CAN_TIMEOUT_MSEC);
            aMotor.config_kI(slotIdx, 0.0, CAN_TIMEOUT_MSEC);
            aMotor.config_kP(slotIdx, 0.0, CAN_TIMEOUT_MSEC);
            aMotor.configClosedLoopPeakOutput(slotIdx, 1.0, CAN_TIMEOUT_MSEC);
            aMotor.configClosedLoopPeriod(slotIdx, 1, CAN_TIMEOUT_MSEC);
            aMotor.configMaxIntegralAccumulator(slotIdx, 0, CAN_TIMEOUT_MSEC);
            aMotor.configAllowableClosedloopError(slotIdx, 0, CAN_TIMEOUT_MSEC);
        }

        // aMotor.configAuxPIDPolarity(false, CAN_TIMEOUT_MSEC);

        aMotor.configOpenloopRamp(0.0, CAN_TIMEOUT_MSEC);
        aMotor.configClosedloopRamp(0.0, CAN_TIMEOUT_MSEC);

        // Start with current limiting disabled and set to a large value
        aMotor.enableCurrentLimit(false);
        aMotor.configContinuousCurrentLimit(90, CAN_TIMEOUT_MSEC);
        aMotor.configPeakCurrentDuration(0, CAN_TIMEOUT_MSEC);
        aMotor.configPeakCurrentLimit(0, CAN_TIMEOUT_MSEC);

        // Assume we have no limit switches associated with this motor
        // aMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, CAN_TIMEOUT_MSEC);
        // aMotor.configForwardSoftLimitEnable(false, CAN_TIMEOUT_MSEC);
        // aMotor.configForwardSoftLimitThreshold(0, CAN_TIMEOUT_MSEC);
        // aMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, CAN_TIMEOUT_MSEC);
        // aMotor.configReverseSoftLimitEnable(false, CAN_TIMEOUT_MSEC);
        // aMotor.configReverseSoftLimitThreshold(0, CAN_TIMEOUT_MSEC);

        // Disable motion magic and profile features until we know we are going to use them
        // aMotor.configMotionAcceleration(0, CAN_TIMEOUT_MSEC);
        // aMotor.configMotionCruiseVelocity(0, CAN_TIMEOUT_MSEC);
        // aMotor.configMotionProfileTrajectoryPeriod(0, CAN_TIMEOUT_MSEC);

        // Reinforce factor default deadband and minimum output
        aMotor.configNeutralDeadband(0.04, CAN_TIMEOUT_MSEC);
        // aMotor.configNominalOutputForward(0.0, CAN_TIMEOUT_MSEC);
        // aMotor.configNominalOutputReverse(0.0, CAN_TIMEOUT_MSEC);

        // aMotor.configPeakOutputForward(1.0, CAN_TIMEOUT_MSEC);
        // aMotor.configPeakOutputReverse(1.0, CAN_TIMEOUT_MSEC);

        // Feedback sources, start with no local and no remote sensors
        // NOTE: Remote sensors can be things like the Pigeon IMU

        // aMotor.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 0, CAN_TIMEOUT_MSEC);        
        for (int pidIdx = 0; pidIdx <= 1; ++pidIdx)
        {
            // aMotor.configSelectedFeedbackCoefficient(1.0, pidIdx, CAN_TIMEOUT_MSEC);
            // aMotor.configSelectedFeedbackSensor(FeedbackDevice.None, pidIdx, CAN_TIMEOUT_MSEC);

            // aMotor.setIntegralAccumulator(0, pidIdx, CAN_TIMEOUT_MSEC);
            // aMotor.setSelectedSensorPosition(0, pidIdx, CAN_TIMEOUT_MSEC);
        }

        // TODO: Figure out what these do!!!!
        //aMotor.configSensorTerm(sensorTerm, feedbackDevice, CAN_TIMEOUT_MSEC);
        //aMotor.configVelocityMeasurementPeriod(period, CAN_TIMEOUT_MSEC);
        //aMotor.configVelocityMeasurementWindow(windowSize, CAN_TIMEOUT_MSEC);

        // aMotor.enableVoltageCompensation(false);
        // aMotor.configVoltageCompSaturation(12.0, CAN_TIMEOUT_MSEC);
        // aMotor.configVoltageMeasurementFilter(32, CAN_TIMEOUT_MSEC);  // Default
        
        // Reinforce control frame period defaults
        // NOTE: Decreasing general control frame period to 1 ms will increase CAN traffic by about 15%
        aMotor.setControlFramePeriod(ControlFrame.Control_3_General, 10);
        // TODO: aMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 10);
        //aMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 10);


        aMotor.setNeutralMode(NeutralMode.Brake);

        // aMotor.setSafetyEnabled(false);
        // aMotor.setExpiration(0.500);

        aMotor.setInverted(false);
        aMotor.setSensorPhase(false);

        // Reinforce the factory defaults that we might change later
        // TODO: To improve CAN bus utilization consider setting the unused one to even lower rates (longer periods)
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,        10, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,      20, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,    160, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,   160, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,    160, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,  160, CAN_TIMEOUT_MSEC);
        // aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,   160, CAN_TIMEOUT_MSEC);

        // aMotor.selectDemandType(false);   // Future feature

        // TODO: Read faults before clearing?
        // aMotor.clearMotionProfileHasUnderrun(CAN_TIMEOUT_MSEC);
        // aMotor.clearStickyFaults(CAN_TIMEOUT_MSEC);

        // aMotor.changeMotionControlFramePeriod(25);  // Assume that any motion profile will step at 50 ms
        // aMotor.clearMotionProfileTrajectories();
        // aMotor.enableHeadingHold(false); // Future feature

        // Use constants from slot 0 and loop 0 (i.e., primary PID) for basic closed loop control
        // See SRM Section 9.10 for more information about the Auxilliary loop
        // aMotor.selectProfileSlot(0, 0);

    }
    /***
     * initialize - performs all post construction initialization This is necessary
     * because some operations cannot be performed until the entire robot has been
     * constructed
     * 
     * Call this function once in robotInit
     */
    public void initialize() 
    {
        // Set up all of the initial smartdashboard items in one place so they are
        // easier to find. Some will only be generated when telemetry is enabled
        // and others are generated in periodic cycles, so these items are the
        // ones that are needed immediately at initialization because they are
        // generated once
        SmartDashboard.putData("DriveStyles", driveStyleChooser);
        SmartDashboard.putBoolean("DriveTelemetryEnabled",false);
        SmartDashboard.putBoolean("TestDriveEnabled",false);
        SmartDashboard.putNumber("LeftTestSpeed", 0.0);
        SmartDashboard.putNumber("RightTestSpeed",0.0);

        // Loop through each motor to configure every setting for deterministic
        // behavior.
        // NOTE: During development it is essential that all settings are pushed
        // to avoid unexpected behaviors. If we were producing a final product
        // to be delivered we would design the system with a separate programming
        // state to push the values; this would save boot time and wear-and-tear
        // of the flash memory
        for (int i = 0; i < leftMotors.length; ++i)
        {
            initializeMotor(leftMotors[i]);
        }
        for (int i = 0; i < rightMotors.length; ++i)
        {
            initializeMotor(rightMotors[i]);
        }

        // Set up follower commands only once to minimize CAN traffic
        for (int i = 1; i < leftMotors.length; ++i) 
        {
            leftMotors[i].follow(leftMotors[0]);
        }
        for (int i = 1; i < rightMotors.length; ++i) 
        {
            rightMotors[i].follow(rightMotors[0]);
        }

        // Configure primary motor sensor device for all feedback loops
        for (int pidIndex = 0; pidIndex <= 0; ++pidIndex)
        {
            ErrorCode errorCode = leftMotors[0].configSelectedFeedbackSensor(FEEDBACK_DEVICE, pidIndex, CAN_TIMEOUT_MSEC);
            leftMotors[1].configSelectedFeedbackSensor(FEEDBACK_DEVICE, pidIndex, CAN_TIMEOUT_MSEC);
            leftMotors[0].setSelectedSensorPosition(0, pidIndex, CAN_TIMEOUT_MSEC);
            leftMotors[1].setSelectedSensorPosition(0, pidIndex, CAN_TIMEOUT_MSEC);
            if (ErrorCode.OK != errorCode)
            {
                // TODO: Insert error message or telemetry status here
            }
            errorCode = rightMotors[0].configSelectedFeedbackSensor(FEEDBACK_DEVICE, pidIndex, CAN_TIMEOUT_MSEC);
            rightMotors[1].configSelectedFeedbackSensor(FEEDBACK_DEVICE, pidIndex, CAN_TIMEOUT_MSEC);
            rightMotors[0].setSelectedSensorPosition(0, pidIndex, CAN_TIMEOUT_MSEC);
            rightMotors[1].setSelectedSensorPosition(0, pidIndex, CAN_TIMEOUT_MSEC);
            if (ErrorCode.OK != errorCode)
            {
                // TODO: Insert error message or telemetry status here
            }            
        }
        leftMotors[0].setSensorPhase(true);

        disable();
    }

    /**
     * disable - put the robot in a known state
     */
    public void disable() 
    {
        drive.stopMotor();
    }

    @Override
    public void initDefaultCommand() 
    {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    /**
     * drive - simplified interface to that selects drive type based on driveStyleChoose
     * @param speed
     * @param turnOrRightSpeed
     */
    public void drive(Joystick control)     // TODO: Not sure I like this, may just want separate functions with speed/turn or left/right args
    {
		double speed = control.getRawAxis(PS4Constants.LEFT_STICK_Y.getValue());
        double turn  = -control.getRawAxis(PS4Constants.RIGHT_STICK_X.getValue());
        double rightSpeed = control.getRawAxis(PS4Constants.RIGHT_STICK_Y.getValue());

        switch (driveStyleChooser.getSelected())
        {
            default:
            case WPI_ARCADE:
            {
                drive.arcadeDrive(0.4*speed, 0.2*turn, false); // Don't square yet, will work on scaler later as new Joystick type
                break;
            }
            case WPI_CURVATURE:
            {
                drive.curvatureDrive(0.4*speed, 0.2*turn, Math.abs(speed)<0.25); // Use quickturn logic when speed is "low"
                break;
            }
            case WPI_TANK:
            {
                drive.tankDrive(0.6*speed, 0.6*rightSpeed);
                break;
            }
            case MODIFIED_ARCADE:
            {
                break;
            }
        }
    }

    private void telemetry()
    {
        leftPosition_ticks = leftMotors[0].getSelectedSensorPosition(0);
        leftVelocity_ticksPsec = leftMotors[0].getSelectedSensorVelocity(0)*10;

        rightPosition_ticks = rightMotors[0].getSelectedSensorPosition(0);
        rightVelocity_ticksPsec = rightMotors[0].getSelectedSensorVelocity(0)*10;

        SmartDashboard.putNumber("leftPosition_ticks", leftPosition_ticks);
        SmartDashboard.putNumber("leftVelocity_ticksPsec", leftVelocity_ticksPsec);
        SmartDashboard.putNumber("rightPosition_ticks", rightPosition_ticks);
        SmartDashboard.putNumber("rightVelocity_ticksPsec", rightVelocity_ticksPsec);

        SmartDashboard.putNumber("leftRearPosition_ticks", leftMotors[1].getSelectedSensorPosition(0));
        SmartDashboard.putNumber("leftRearVelocity_ticksPsec", leftMotors[1].getSelectedSensorVelocity(0)*10);
        SmartDashboard.putNumber("rightRearPosition_ticks", rightMotors[1].getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightRearVelocity_ticksPsec", rightMotors[1].getSelectedSensorVelocity(0)*10);

        for (int i = 0; i < leftMotors.length; ++i) 
        {
            leftCurrent_amp[i] = leftMotors[i].getOutputCurrent();
            SmartDashboard.putNumber("leftCurrent_amp"+Integer.toString(i), leftCurrent_amp[i]);
        }
        for (int i = 0; i < rightMotors.length; ++i) 
        {
            rightCurrent_amp[i] = rightMotors[i].getOutputCurrent();
            SmartDashboard.putNumber("rightCurrent_amp"+Integer.toString(i), rightCurrent_amp[i]);
        }
    }

    @Override
    public void periodic() 
    {
        if (SmartDashboard.getBoolean("DriveTelemetryEnabled",false))
        {
            telemetry();
        }
        if (SmartDashboard.getBoolean("TestDriveEnabled",false))
        {
            double leftSpeed = SmartDashboard.getNumber("LeftTestSpeed", 0.0);
            double rightSpeed = SmartDashboard.getNumber("RightTestSpeed",0.0);
            leftMotors[0].set(leftSpeed);
            rightMotors[0].set(rightSpeed);
        }
    }
}
