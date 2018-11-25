/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drivesubsystem;

import frc.robot.OI;
import frc.robot.PS4Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.apache.commons.math3.stat.descriptive.DescriptiveStatistics;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private WPI_TalonSRX[] leftMotors;
    private WPI_TalonSRX[] rightMotors;

    private final int NUM_MOTORS;
    private boolean followModeEnabled = false;

    private double timeOfSampleStart_sec;
    private int velSampleIndex;
    private int numVelSamples;
    private int[][] leftVelocitySamples_ticksP100ms;
    private int[][] rightVelocitySamples_ticksP100ms;

    private final int NUM_VELOCITY_SAMPLES;

    private DescriptiveStatistics descriptiveStatistics;

    private DifferentialDrive drive;

    private enum DriveStyles 
    {
        WPI_ARCADE, 
        WPI_CURVATURE, 
        WPI_TANK, 
        OPEN_LOOP_SAMPLE,
        TEST_VEL_MODE,
    };

    private SendableChooser<DriveStyles> driveStyleChooser;

    /***
     * Constructor
     */
    public DriveSubsystem() 
    {
        assert RobotMap.DRIVE_MOTOR_LEFT_ID_GROUP.length == RobotMap.DRIVE_MOTOR_RIGHT_ID_GROUP.length : "Motor Count Must Be Symmetrical";
        NUM_MOTORS = RobotMap.DRIVE_MOTOR_LEFT_ID_GROUP.length;

        driveStyleChooser = new SendableChooser<DriveStyles>(); // put this on the dashboard at initialization
        driveStyleChooser.setDefaultOption("WPI Arcade",    DriveStyles.WPI_ARCADE);
        driveStyleChooser.addOption( "WPI Curvature",       DriveStyles.WPI_CURVATURE);
        driveStyleChooser.addOption( "WPI Tank",            DriveStyles.WPI_TANK);
        driveStyleChooser.addOption( "Open Loop Sample",    DriveStyles.OPEN_LOOP_SAMPLE);
        driveStyleChooser.addOption( "Test Vel Mode",       DriveStyles.TEST_VEL_MODE);
        
        // Configure motors at initialiation
        leftMotors  = new WPI_TalonSRX[NUM_MOTORS];
        rightMotors = new WPI_TalonSRX[NUM_MOTORS];

        NUM_VELOCITY_SAMPLES = (int)(3.0 * RobotMap.ROBOT_RATE_HZ);
        leftVelocitySamples_ticksP100ms  = new int[NUM_MOTORS][NUM_VELOCITY_SAMPLES];
        rightVelocitySamples_ticksP100ms = new int[NUM_MOTORS][NUM_VELOCITY_SAMPLES];
        timeOfSampleStart_sec = 0.0;

        descriptiveStatistics = new DescriptiveStatistics();

        for (int i = 0; i < NUM_MOTORS; ++i) 
        {
            leftMotors[i] = new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_LEFT_ID_GROUP[i]);
            leftMotors[i].setName(getName(),"Left_" + Integer.toString(i));

            rightMotors[i] = new WPI_TalonSRX(RobotMap.DRIVE_MOTOR_RIGHT_ID_GROUP[i]);
            rightMotors[i].setName(getName(),"Right_" + Integer.toString(i));            
        }

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
    private void initializeMotorDefaults(WPI_TalonSRX aMotor)
    {
        // TODO: Check ErrorCode?

        aMotor.stopMotor();

        // Group all of the slot configurations together for a loop
        for (int slotIdx = 0; slotIdx < 2; ++slotIdx)   // TODO: Source MAGIC NUMBER 2, what is the actual number of slots in the firmware?
        {
            aMotor.config_IntegralZone(slotIdx, 0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.config_kF(slotIdx, 0.0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.config_kP(slotIdx, 0.0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.config_kI(slotIdx, 0.0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.config_kD(slotIdx, 0.0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.configClosedLoopPeakOutput(slotIdx, 1.0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.configClosedLoopPeriod(slotIdx, 1, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.configMaxIntegralAccumulator(slotIdx, 0, RobotMap.CAN_TIMEOUT_MSEC);
            aMotor.configAllowableClosedloopError(slotIdx, 0, RobotMap.CAN_TIMEOUT_MSEC);
        }

        // aMotor.configAuxPIDPolarity(false, RobotMap.CAN_TIMEOUT_MSEC);

        aMotor.configOpenloopRamp(0.0, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.configClosedloopRamp(0.0, RobotMap.CAN_TIMEOUT_MSEC);

        // Start with current limiting disabled and set to a large value
        aMotor.enableCurrentLimit(false);
        aMotor.configContinuousCurrentLimit(90, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.configPeakCurrentLimit(0, RobotMap.CAN_TIMEOUT_MSEC);

        // Assume we have no limit switches associated with this motor
        // aMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configForwardSoftLimitEnable(false, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configForwardSoftLimitThreshold(0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configReverseSoftLimitEnable(false, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configReverseSoftLimitThreshold(0, RobotMap.CAN_TIMEOUT_MSEC);

        // Disable motion magic and profile features until we know we are going to use them
        // aMotor.configMotionAcceleration(0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configMotionCruiseVelocity(0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configMotionProfileTrajectoryPeriod(0, RobotMap.CAN_TIMEOUT_MSEC);

        // Reinforce factor default deadband and minimum output
        aMotor.configNeutralDeadband(0.04, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configNominalOutputForward(0.0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configNominalOutputReverse(0.0, RobotMap.CAN_TIMEOUT_MSEC);

        // aMotor.configPeakOutputForward(1.0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configPeakOutputReverse(1.0, RobotMap.CAN_TIMEOUT_MSEC);

        // Feedback sources, start with no local and no remote sensors
        // NOTE: Remote sensors can be things like the Pigeon IMU

        // aMotor.configRemoteFeedbackFilter(0, RemoteSensorSource.Off, 0, RobotMap.CAN_TIMEOUT_MSEC);        
        for (int pidIdx = 0; pidIdx <= 1; ++pidIdx)
        {
            // aMotor.configSelectedFeedbackCoefficient(1.0, pidIdx, RobotMap.CAN_TIMEOUT_MSEC);
            // aMotor.configSelectedFeedbackSensor(FeedbackDevice.None, pidIdx, RobotMap.CAN_TIMEOUT_MSEC);

            // aMotor.setIntegralAccumulator(0, pidIdx, RobotMap.CAN_TIMEOUT_MSEC);
            // aMotor.setSelectedSensorPosition(0, pidIdx, RobotMap.CAN_TIMEOUT_MSEC);
        }

        // TODO: Figure out what these do!!!!
        //aMotor.configSensorTerm(sensorTerm, feedbackDevice, RobotMap.CAN_TIMEOUT_MSEC);
        //aMotor.configVelocityMeasurementPeriod(period, RobotMap.CAN_TIMEOUT_MSEC);
        //aMotor.configVelocityMeasurementWindow(windowSize, RobotMap.CAN_TIMEOUT_MSEC);

        // aMotor.enableVoltageCompensation(false);
        // aMotor.configVoltageCompSaturation(12.0, RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.configVoltageMeasurementFilter(32, RobotMap.CAN_TIMEOUT_MSEC);  // Default
        
        // Reinforce control frame period defaults
        // NOTE: Decreasing general control frame period to 1 ms will increase CAN traffic by about 15%
        aMotor.setControlFramePeriod(ControlFrame.Control_3_General, 10);
        // TODO: aMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 10);
        //aMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 10);


        aMotor.setNeutralMode(NeutralMode.Brake);

        aMotor.setSafetyEnabled(false);
        aMotor.setExpiration(0.500);

        aMotor.setInverted(false);
        aMotor.setSensorPhase(false);

        // Reinforce the factory defaults that we might change later
        // To improve CAN bus utilization consider setting the unused one to even lower rates (longer periods)
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,        10, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,      20, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,    160, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,   160, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,    160, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,  160, RobotMap.CAN_TIMEOUT_MSEC);
        aMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,   160, RobotMap.CAN_TIMEOUT_MSEC);

        // aMotor.selectDemandType(false);   // Future feature

        // TODO: Read faults before clearing?
        // aMotor.clearMotionProfileHasUnderrun(RobotMap.CAN_TIMEOUT_MSEC);
        // aMotor.clearStickyFaults(RobotMap.CAN_TIMEOUT_MSEC);

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
        // There are times when we won't send any commands to the motor and we would
        // like to suppress the motor safety logic to prevent complaints.
        // TODO: We may consider changing this at some point to ensure the robot does
        // not keep moving if there is a timing problem
        drive.setSafetyEnabled(false);

        // Don't let the drive class maintain the inversion
        // Instead we will control the inversion state directly
        // within the motor controller instances so we can keep
        // the polarity and sensor phase sensible for all operating
        // modes
        drive.setRightSideInverted(false);

        // Set up all of the initial smartdashboard items in one place so they are
        // easier to find. Some will only be generated when telemetry is enabled
        // and others are generated in periodic cycles, so these items are the
        // ones that are needed immediately at initialization because they are
        // generated once
        SmartDashboard.putData("DriveStyles", driveStyleChooser);
        SmartDashboard.putBoolean("DriveTelemetryEnabled",false);
        SmartDashboard.putBoolean("DriveDiagnosticsEnabled",false);
        SmartDashboard.putNumber("TestVelocity_rpm",100.0);

        // Loop through each motor to configure every setting for deterministic
        // behavior.
        // NOTE: During development it is essential that all settings are pushed
        // to avoid unexpected behaviors. If we were producing a final product
        // to be delivered we would design the system with a separate programming
        // state to push the values; this would save boot time and wear-and-tear
        // of the flash memory
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            initializeMotorDefaults(leftMotors[i]);
            initializeMotorDefaults(rightMotors[i]);    
        }

        // Configure the motors for this robot drive subsystem
        // NOTE: If the number of controllers is > 1 then some things will be set
        // up more than once; minor impact to performance and flash longevity
        for (int pidIndex = 0; pidIndex < RobotMap.NUM_CONTROLLER_FEEDBACK_LOOPS; ++pidIndex)
        {
            for (int i = 0; i < NUM_MOTORS; ++i)
            {
                // LEFT MOTORS get the left motor constants
                leftMotors[i].setInverted(RobotMap.DRIVE_LEFT_MOTOR_INVERSION);
                leftMotors[i].configSelectedFeedbackSensor(RobotMap.DRIVE_MOTOR_FEEDBACK_DEVICE, 
                                                           pidIndex, 
                                                           RobotMap.CAN_TIMEOUT_MSEC);
                leftMotors[i].setSelectedSensorPosition(0,
                                                        pidIndex, 
                                                        RobotMap.CAN_TIMEOUT_MSEC);
                leftMotors[i].setSensorPhase(RobotMap.DRIVE_LEFT_SENSOR_PHASE);
                leftMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,    
                                                   RobotMap.DRIVE_QUADRATURE_STATUS_FRAME_PERIOD_MSEC, 
                                                   RobotMap.CAN_TIMEOUT_MSEC);

                // RIGHT MOTORS get the right motor constants
                rightMotors[i].setInverted(RobotMap.DRIVE_RIGHT_MOTOR_INVERSION); 
                rightMotors[i].configSelectedFeedbackSensor(RobotMap.DRIVE_MOTOR_FEEDBACK_DEVICE, 
                                                            pidIndex, 
                                                            RobotMap.CAN_TIMEOUT_MSEC);
                rightMotors[i].setSelectedSensorPosition(0, 
                                                         pidIndex, 
                                                         RobotMap.CAN_TIMEOUT_MSEC);
                rightMotors[i].setSensorPhase(RobotMap.DRIVE_RIGHT_SENSOR_PHASE);
                rightMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,    
                                                    RobotMap.DRIVE_QUADRATURE_STATUS_FRAME_PERIOD_MSEC, 
                                                    RobotMap.CAN_TIMEOUT_MSEC);
            }
        }

        leftMotors[0].config_kF(0, 0.07764705882, RobotMap.CAN_TIMEOUT_MSEC);
        //leftMotors[0].config_kP(0, 0.0525, RobotMap.CAN_TIMEOUT_MSEC);

        leftMotors[1].config_kF(0, 0.09007660474, RobotMap.CAN_TIMEOUT_MSEC);
        //leftMotors[1].config_kP(0, 0.0445, RobotMap.CAN_TIMEOUT_MSEC);


        rightMotors[0].config_kF(0, 0.09305075496, RobotMap.CAN_TIMEOUT_MSEC);
        rightMotors[0].config_kP(0, 0.1411, RobotMap.CAN_TIMEOUT_MSEC);

        rightMotors[1].config_kF(0, 0.08079930495, RobotMap.CAN_TIMEOUT_MSEC);
        rightMotors[1].config_kP(0, 0.2728/2, RobotMap.CAN_TIMEOUT_MSEC);

        // Set up follower commands only once to minimize CAN traffic
        // when not in closed loop control
        enableFollowMode();

        disable();
    }

    /**
     * disable - put the robot in a known state
     */
    public void disable() 
    {
        drive.stopMotor();
    }

    /**
     * enableFollowMode - enables following modes if not already enabled
     */
    private void enableFollowMode()
    {
        if (!followModeEnabled)
        {
            for (int i = 1; i < NUM_MOTORS; ++i)
            {
                leftMotors[i].follow(leftMotors[0]);
                rightMotors[i].follow(rightMotors[0]);
            }
            followModeEnabled = true;  
        }       
    }

    /**
     * disableFollowMode - disables following modes if not already disabled
     */
    private void disableFollowMode()
    {
        if (followModeEnabled)
        {
            for (int i = 1; i < NUM_MOTORS; ++i)
            {
                leftMotors[i].set(0.0);
                rightMotors[i].set(0.0);
            }
            followModeEnabled = false;
        }
    }

    @Override
    public void initDefaultCommand() 
    {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    /**
     * drive - simplified interface to that selects drive type based on driveStyleChoose
     */
    public void drive()
    {
		double speed = OI.speed();
        double turn  = OI.turn();
        double rightSpeed = OI.rightSpeed();

        switch (driveStyleChooser.getSelected())
        {
            default:
            case WPI_ARCADE:
            {
                enableFollowMode();
                drive.arcadeDrive(RobotMap.DRIVE_SPEED_LIMIT_FACTOR*speed, 
                                  RobotMap.DRIVE_TURN_LIMIT_FACTOR*turn, 
                                  false); // Don't square yet, will work on scaler later as new Joystick type
                break;
            }
            case WPI_CURVATURE:
            {
                enableFollowMode();
                drive.curvatureDrive(RobotMap.DRIVE_SPEED_LIMIT_FACTOR*speed, 
                                     RobotMap.DRIVE_TURN_LIMIT_FACTOR*turn, 
                                     Math.abs(turn)>0.8); // Use quickturn logic when user cranks it over
                break;
            }
            case WPI_TANK:
            {
                enableFollowMode();
                drive.tankDrive(RobotMap.DRIVE_TANK_LIMIT_FACTOR*speed, 
                                RobotMap.DRIVE_TANK_LIMIT_FACTOR*rightSpeed);
                break;
            }
            case OPEN_LOOP_SAMPLE:
            {
                if (OI.testButton())
                {
                    // Set all motors to full speed
                    // Keep sampling until user stops pressing button
                    if (timeOfSampleStart_sec == 0.0)
                    {
                        // Initialize time to track when sampling should
                        // really begin and end
                        timeOfSampleStart_sec = Timer.getFPGATimestamp();
                        velSampleIndex = 0;
                        numVelSamples = 0;
                    }
                    for (int i = 0; i < NUM_MOTORS; ++i)
                    {
                        leftMotors[i].set(1.0);
                        rightMotors[i].set(1.0);
                    }

                    // Wait one second before starting to sample
                    if (Timer.getFPGATimestamp() > (timeOfSampleStart_sec + 1.0))
                    {
                        for (int i = 0; i < NUM_MOTORS; ++i)
                        {
                            leftVelocitySamples_ticksP100ms[i][velSampleIndex] = leftMotors[i].getSelectedSensorVelocity(0);
                            rightVelocitySamples_ticksP100ms[i][velSampleIndex] = rightMotors[i].getSelectedSensorVelocity(0);
                            if (numVelSamples < NUM_VELOCITY_SAMPLES)
                            {
                                ++numVelSamples;
                            }
                        }
                        // Just override the last sample if we have enough samples
                        if (velSampleIndex < NUM_VELOCITY_SAMPLES-1)
                        {
                            velSampleIndex += 1;
                        }
                        else
                        {
                            velSampleIndex = 0;
                        }
                    }


                }
                else
                {
                    timeOfSampleStart_sec = 0.0;
                    if (numVelSamples >= 100)
                    {
                        // We have enough samples to make an estimate
                        SmartDashboard.putNumber("velocitySamples", numVelSamples);
                        for (int i = 0; i < NUM_MOTORS; ++i)
                        {
                            String stri = Integer.toString(i);

                            // Add the data from the left array
                            descriptiveStatistics.clear();
                            for( int j = 0; j < numVelSamples; ++j) 
                            {
                                descriptiveStatistics.addValue(leftVelocitySamples_ticksP100ms[0][i]);
                            }

                            // Compute some statistics
                            SmartDashboard.putNumber("leftMeanVelocity_tickP100ms_"+stri, descriptiveStatistics.getMean());
                            SmartDashboard.putNumber("leftStdVelocity_tickP100ms_"+stri, descriptiveStatistics.getStandardDeviation());
                            SmartDashboard.putNumber("leftMedianVelocity_tickP100ms_"+stri, descriptiveStatistics.getPercentile(50));

                            // Add the data from the left array
                            descriptiveStatistics.clear();
                            for( int j = 0; j < numVelSamples; ++j) 
                            {
                                descriptiveStatistics.addValue(rightVelocitySamples_ticksP100ms[0][i]);
                            }

                            // Compute some statistics
                            SmartDashboard.putNumber("rightMeanVelocity_tickP100ms_"+stri, descriptiveStatistics.getMean());
                            SmartDashboard.putNumber("rightStdVelocity_tickP100ms_"+stri, descriptiveStatistics.getStandardDeviation());
                            SmartDashboard.putNumber("rightMedianVelocity_tickP100ms_"+stri, descriptiveStatistics.getPercentile(50));

                        }
                        numVelSamples = 0;                        
                    }
                    enableFollowMode();
                    drive.stopMotor();
                }
                break;
            }
            case TEST_VEL_MODE:
            {
                if (OI.testButton())
                {
                    disableFollowMode();
                    double rpm = SmartDashboard.getNumber("TestVelocity_rpm",100.0);
                    double ticksP100ms = RobotMap.driveRpmToTicks(rpm);
                    SmartDashboard.putNumber("TestVelocityCommand_ticksP100ms", ticksP100ms);

                    for (int i = 0; i < NUM_MOTORS; ++i)
                    {
                        leftMotors[i].set(ControlMode.Velocity,   ticksP100ms);
                        rightMotors[i].set(ControlMode.Velocity,  ticksP100ms);
                    }
                }
                else
                {
                    enableFollowMode();
                    drive.stopMotor();
                }
                break;
            }
        }
    }

    private void telemetry()
    {
        for (int i = 0; i < NUM_MOTORS; ++i) 
        {
            String stri = Integer.toString(i);

            SmartDashboard.putNumber("leftCurrent_amp_"+stri,            leftMotors[i].getOutputCurrent());
            SmartDashboard.putNumber("leftPosition_ticks_"+stri,         leftMotors[i].getSelectedSensorPosition(0));
            SmartDashboard.putNumber("leftVelocity_ticksP100ms_"+stri,   leftMotors[i].getSelectedSensorVelocity(0));
            SmartDashboard.putNumber("leftClosedError_"+stri,            leftMotors[i].getClosedLoopError(0));

            SmartDashboard.putNumber("rightCurrent_amp_"+stri,           rightMotors[i].getOutputCurrent());
            SmartDashboard.putNumber("rightPosition_ticks_"+stri,        rightMotors[i].getSelectedSensorPosition(0));
            SmartDashboard.putNumber("rightVelocity_ticksP100ms_"+stri,  rightMotors[i].getSelectedSensorVelocity(0));
            SmartDashboard.putNumber("rightClosedError_"+stri,           rightMotors[i].getClosedLoopError(0));
        }
    }

    @Override
    public void periodic() 
    {
        if (SmartDashboard.getBoolean("DriveTelemetryEnabled",false))
        {
            telemetry();
        }
    }
}
