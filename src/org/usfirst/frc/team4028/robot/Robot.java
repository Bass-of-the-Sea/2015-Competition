package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.RobotData.InputData;
import org.usfirst.frc.team4028.robot.RobotData.OutputData;
import org.usfirst.frc.team4028.robot.RobotData.WorkingData;
import org.usfirst.frc.team4028.robot.Constants.LogitechF310;
import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.Constants.RobotMap.*;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.time.ZonedDateTime;
import java.util.Date;
import java.util.TimeZone;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 * 
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 4.Nov.2015	0.84	Sebastian Rodriguez			Further Updated Button Mapping to match Labview code
 * 
 * 27.Oct.2015	0.83	Tom Bruns					2nd Robot Test, Updated Robot Mapping
 * 						Sebastian Rodriguez
 * 
 * 17.Oct.2015	0.82	Tom Bruns					Implemented Lift Hold Position feature
 * 
 * 05.Oct.2015	0.81	Sebastian Rodriguez			Troubleshoot 1st deployment to the real robot
 * 						 Tom Bruns		
 * 
 * 30.Aug.2015	0.8		Sebastian Rodriguez			Add addl NavX values to SmartDashboard
 * 						 Tom Bruns					
 * 
 * 23.Aug.2015	0.71	Tom Bruns					Re-factored robot constants to RobotMap.java
 * 													
 * 22.Aug.2015	0.7		Sebastian Rodriguez			Got DIO working
 * 						 Tom Bruns					Initial Integration of Navigation Sensor
 * 
 * 16.Aug.2015	0.6		Sebastian Rodriguez			Code Review, familiarization
 * 						 Adam Cool					
 * 						 Tom Bruns
 * 
 * 08.Aug.2015	0.5		Tom Bruns					Added Addl Data logging variables
 * 													Re-layed out SmartDashboard
 * 
 * 02.Aug.2015	0.4		Tom Bruns					Implemented text file logging
 * 													Implemented user selected Auton Mode
 * 													Structured code to support multiple Autom modes
 * 
 * 01.Aug.2015	0.3		Tom Bruns					Added de-bounce to speed scaling factor btns
 * 													set invert on right drive axis feedback sensor
 * 													added invert of DIO sensor
 * 													Refactor to use RobotData class in Telop and Auton
 * 													initial work on using deceleration to stop at target
 *
 * 25.Jul.2015	0.2		Tom Bruns					Refactoring: implement variable naming conventions
 * 																 added many comments
 * 																 initial support for Smart Dashboard
 * 															  	 implement speed scaling from gamepad button (25%, 50%, 75%, 100%)
 * 
 * 27.Jun.2015	0.1		Sebastian Rodriguez			Initial Version:	Added Logitech Gamepad constants
 * 						 Tom Bruns										Defined Robot Config
 * 																		Initial Telop and Autom methods
 * 																 
 * Overview:
 * 	The goal of this code is to re-implement the 2015 Season "Recycle Rush" control code in JAVA
 * 		as a technology POC (Proof Of Concept) and Test Bed.
 * 
 * 	The Controls Team is evaluating a language change from LabView to Java and we want to understand what
 * 		the level of effort and complexity if we propose this change. 
 * 
*/

public class Robot extends IterativeRobot 
{
	// ======================================
	// define constants for Auton Modes (used by Smart Dashboard to allow user selection prior to auton init)
	// ======================================
	public enum AutonMode 
	{
	    UNDEFINED, 
	    DO_NOTHING, 
	    SIMPLE, 
	    COMPLEX
	}
	
	// ===========================================================
	//   Define class level instance variables for Robot Runtime controllable objects  
	// ===========================================================
	
	private RobotDrive _ourRobot;
	
	// Driver & Operator station gamepads
	private Joystick _driverGamepad;
	private Joystick _operatorGamepad;

	// CIM DC Motors on Talon SRX Speed Controllers (via CAN Bus)
	private CANTalon _liftMasterMtr;
	private CANTalon _liftSlaveMtr;
	private CANTalon _leftDriveMasterMtr;
	private CANTalon _leftDriveSlaveMtr;
	private CANTalon _rightDriveMasterMtr;
	private CANTalon _rightDriveSlaveMtr;
	
	// CIM DC Motors on Victor SP Speed Controllers (via PWM Ports)
	private VictorSP _leftInfeedMtr;
	private VictorSP _rightInfeedMtr;
	//private TalonSRX _testMtr;
	
	// DC Servo Motor (only used for testing, not on real robot)
	//private Servo _hitecServoMtr;
	
	// Pneumatic Solenoids for Air Cylinders
	private DoubleSolenoid _canClamp;		
	private DoubleSolenoid _frontClips;	
	
	// DIO Devices / Sensors
	private DigitalInput _toteInPositionSensor;
	
	// Arcade Drive with four drive motors
	private RobotDrive _robotDrive;
	
	private AHRS _navXSensor;
		
	// ===========================================================
	//   Define class level working variables 
	// ===========================================================
	int _autoLoopCounter;
		
	// DTO (Data Transfer Object) holding all live Robot Data Values
	RobotData _robotLiveData;

	// wrapper around data logging (if it is enabled)
	DataLogger _dataLogger;
	
	// Smart Dashboard Chooser
	SendableChooser autonModeChooser;
	
    /*****************************************************************************************************
     * This function is run when the robot is first started up.
	 * This is where we initialize the robot hardware configuration.
	 * 
	 * We try and fully configure the Motor controllers each robot startup.
	 *  We are as explicit as possible even when we do not need to be to make it as clear as possible for others
	 * 	This way we do not assume what their current configuration is.
	 * 	The only thing we depend on is that the CAN Bus address is correct
	 * 
	 * FYI:	Additional relevant documentation about each control object is included here
     *****************************************************************************************************/
    public void robotInit() 
    {    	
    	//myRobot = new RobotDrive(0,1);
    	
    	// ===================
    	// Lift Motors, Tandem Pair, looking out motor shaft: CW = UP 
    	//		(Caution: The output shaft of a gearbox can be reversed from the motors !)
    	//					We really care about what direction of the motors vs direction of movement by the axis
        // 
    	// In open loop mode to control the motor speed....	
    	//	ex: _liftMasterMtr.set(liftMtrAdjVelocityCmd);
    	// 			This takes a number from -1 (100% speed in reverse) to +1 (100% speed going forward)
    	// ===================
    	_liftMasterMtr = new CANTalon(RobotMap.CAN_ADDR_LIFT_MASTER_MTR);
    	_liftMasterMtr.changeControlMode(CANTalon.ControlMode.PercentVbus);	// open loop throttle
    	_liftMasterMtr.ConfigFwdLimitSwitchNormallyOpen(false);				// default to Normally Closed Limit Switch 
    																		//		Closed = Axis is NOT ON Limit 
    																		//		Forward = (+) or Top EOT
    	_liftMasterMtr.ConfigRevLimitSwitchNormallyOpen(false);				// default to Normally Closed Limit Switch
    																		//		Closed = Axis is NOT ON Limit 
    																		//		Reverse = (-) or Bottom EOT
    	_liftMasterMtr.enableBrakeMode(true);								// default to brake mode ENABLED
    	_liftMasterMtr.enableLimitSwitch(true, true);						// default to limit switches ENABLED
    	_liftMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);		// set encoder to be feedback device
    	_liftMasterMtr.reverseSensor(false);  								// do not invert encoder feedback
    	
    	_liftSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_LIFT_SLAVE_MTR);				
    	_liftSlaveMtr.changeControlMode(CANTalon.ControlMode.Follower);		// set this mtr ctrlr as a slave
    	_liftSlaveMtr.set(RobotMap.CAN_ADDR_LIFT_MASTER_MTR);			
    	_liftSlaveMtr.ConfigFwdLimitSwitchNormallyOpen(true);				// default to Normally Closed Limit Switch
    	_liftSlaveMtr.ConfigRevLimitSwitchNormallyOpen(true);				// default to Normally Closed Limit Switch
    	_liftSlaveMtr.enableBrakeMode(true);								// default to brake mode ENABLED
    	_liftSlaveMtr.enableLimitSwitch(false, false);						// default to limit switches DISABLED
    																		//	(they are connected to the master (other) talon)
    	
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_MTR);
    	_leftDriveMasterMtr.changeControlMode(CANTalon.ControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_leftDriveMasterMtr.enableLimitSwitch(false, false);				// default to limit switches DISABLED
    																		//	(no L/S on this axis)
    	_leftDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_leftDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
    	
    	_leftDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_LEFT_DRIVE_SLAVE_MTR);	
    	_leftDriveSlaveMtr.changeControlMode(CANTalon.ControlMode.Follower);	// set this mtr ctrlr as a slave
    	_leftDriveSlaveMtr.set(RobotMap.CAN_ADDR_LEFT_DRIVE_MASTER_MTR);
    	_leftDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_leftDriveSlaveMtr.enableLimitSwitch(false, false);					// default to limit switches DISABLED
    	
    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CCW = Drive FWD
    	// ===================
    	_rightDriveMasterMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_MTR);
    	_rightDriveMasterMtr.changeControlMode(CANTalon.ControlMode.PercentVbus);	// open loop throttle.
    	_rightDriveMasterMtr.enableBrakeMode(false);						// default to brake mode DISABLED
    	_rightDriveMasterMtr.enableLimitSwitch(false, false);				// default to limit switches DISABLED
    																		//    	(no L/S on this axis)
    	_rightDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_rightDriveMasterMtr.reverseSensor(true);  							// invert encoder feedback
    	
    	_rightDriveSlaveMtr = new CANTalon(RobotMap.CAN_ADDR_RIGHT_DRIVE_SLAVE_MTR);	
    	_rightDriveSlaveMtr.changeControlMode(CANTalon.ControlMode.Follower);	// set this mtr ctrlr as a slave
    	_rightDriveSlaveMtr.set(RobotMap.CAN_ADDR_RIGHT_DRIVE_MASTER_MTR);
    	_rightDriveSlaveMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_rightDriveSlaveMtr.enableLimitSwitch(false, false);				// default to limit switches DISABLED
    	
    	// ===================
    	// Infeed Motors
    	// ===================
    	_leftInfeedMtr = new VictorSP(RobotMap.PWM_PORT_LEFT_INFEED_MOTOR);			// looking out shaft: CW = Infeed
    	_rightInfeedMtr = new VictorSP(RobotMap.PWM_PORT_RIGHT_INFEED_MOTOR);		// looking out shaft: CCW = Infeed

    	// ===================
    	// Servo Motors
    	// ===================
    	//_hitecServoMtr = new Servo(RobotMap.PWM_PORT_HITEC_SERVO_MOTOR);				// extra motor just for testing
    	   
    	//_testMtr = new TalonSRX(RobotMap.PWM_PORT_TEST_MOTOR);
    	
    	// ===================
    	// Gamepads
    	// ===================
    	_driverGamepad = new Joystick(RobotMap.DRIVER_GAMEPAD_USB_PORT);				// std Logitech F310 Gamepad 
    	_operatorGamepad = new Joystick(RobotMap.OPERATOR_GAMEPAD_USB_PORT);			// std Logitech F310 Gamepad
    	
    	
    	// ===================
    	// Arcade Drive
    	//====================
    	_robotDrive = new RobotDrive(_leftDriveMasterMtr,_rightDriveMasterMtr);
    	// Arcade Drive configured to drive in two motor setup, other two motors follow as slaves
    	
    	// ===================
    	// Air Cylinders
    	// ===================
    	// Can Clamp Air Cylinder is std dual action (actuated in / actuated out)
    	//	In (Retract) = OPEN Clamp
    	//	Out (Extend) = CLOSE Clamp
    	_canClamp = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_CAN_CLAMP_EXTEND, RobotMap.PCM_PORT_CAN_CLAMP_RETRACT);
    	// Clip Air Cylinders are single action (string return in / actuated out )
    	//	In (Retract) = CLOSE Clips
    	//	Out (Extend) = OPEN Clips
    	_frontClips = new DoubleSolenoid(RobotMap.CAN_ADDR_PCM, RobotMap.PCM_PORT_FRONT_CLIP_EXTEND, RobotMap.PCM_PORT_FRONT_CLIP_RETRACT);
    	
    	// ===================
    	// Sensors
    	// ===================
    	// Adjustable Infrared Sensor (E18-D80NK-N), yellow body, 3 wire
    	//	Sensor is N/O and RoboRIO DIO port are pulled high (pg 15 of RoboRIO manual) 
    	//		so	Tote NOT in position 	=> Switch Open		=> DIO = True  
    	//			Tote In position		=> Switch Closed	=> DIO = False 
    	_toteInPositionSensor = new DigitalInput(RobotMap.DIO_PORT_TOTE_IN_POSITION_SENSOR);
    	
    	try 
    	{
            /* Communicate w/ navX MXP via one of the following ports                           	*/
            /*   				I2C.Port.kMXP, 												   	*/
            /* 					SerialPort.Port.kMXP, 										   	*/
            /* 					SerialPort.Port.kUSB										   	*/
            /* 					SPI.Port.kMXP   			plugged into mxp port on RoboRio	*/			
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.  */
            _navXSensor = new AHRS(SPI.Port.kMXP);
            
            DriverStation.reportError("..navX sensor connected" , false);
        } 
    	catch (RuntimeException ex ) 
    	{
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    	
    	// ===================
    	// Smart Dashboard User Input
    	// ===================
    	autonModeChooser = new SendableChooser();
    	autonModeChooser.addDefault("Do Nothing", AutonMode.DO_NOTHING);
    	autonModeChooser.addObject("Simple", AutonMode.SIMPLE);
    	autonModeChooser.addObject("Complex", AutonMode.COMPLEX);
    	SmartDashboard.putData("Autonomous mode chooser", autonModeChooser);
    	
    	// write jar build d&t to the dashboard
    	try
    	{
    		//DriverStation.reportError("** Team 4028 The Beak Squad **", false);
    		
    		//get the path of the currently executing jar file
			String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();		
			//DriverStation.reportError(currentJarFilePath , false);
			Path filePath = Paths.get(currentJarFilePath);
			
			//get file system details from current file
			BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
			Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());
	
			// convert from UTC to local time zone
			SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
			outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern")); 
			String newDateString = outputFormatter.format(utcFileDate);
			
			// write the build date & time to the operator's console log window
			DriverStation.reportError("Build Date and Time: " + newDateString, false);
			
		} 
    	catch (URISyntaxException e) 
    	{
    		DriverStation.reportError("Error determining filename of current JAR file", true);
			//e.printStackTrace();
		} 
    	catch (IOException e) 
    	{	
    		DriverStation.reportError("General Error trying to determine current JAR file", true);
			//e.printStackTrace();
		}	
    }
    
    /*
     * ***************************************************************************************************
     * This function is run 1x each time the robot enters autonomous mode
     *  (Setup the initial robot state for auton mode in this method)
     *****************************************************************************************************
     */
    public void autonomousInit() 
    {
    	// init the loop ctr
    	_autoLoopCounter = 0;
    	
    	_robotLiveData = new RobotData();
    	// this property can be used to track what "State Machine State" the code thinks the robot is in
    	// in init we default to state "00"
    	_robotLiveData.WorkingDataValues.CurrentAutonState = "S-00";
    	
    	// set our desired default state for the can clamp and front clips
    	_robotLiveData.OutputDataValues.CanClampPosition = RobotMap.CAN_CLAMP_OPEN_POSITION;
    	_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	
    	// enable limit switches on tote lift axis (in case they were previously manually overridden)
    	_liftMasterMtr.enableLimitSwitch(true, true);
    	
    	// turn off rumble (only works on XBox Gamepad Controllers)
    	_operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
    	
    	// initialize axis (Encoder) positions
    	_liftMasterMtr.setPosition(0);			// TODO: this should really only happen at home position
    	_leftDriveMasterMtr.setPosition(0);
    	_rightDriveMasterMtr.setPosition(0);
    	
    	// get initial values from Encoders
    	_robotLiveData.WorkingDataValues.LiftEncoderInitialCount = _liftMasterMtr.getPosition();
    	_robotLiveData.WorkingDataValues.LeftDriveEncoderInitialCount = _leftDriveMasterMtr.getPosition();
    	_robotLiveData.WorkingDataValues.RightDriveEncoderInitialCount = _rightDriveMasterMtr.getPosition();
    	
    	// get user input values from the Smart Dashboard
    	_robotLiveData.InputDataValues.AutonModeRequested = (AutonMode) autonModeChooser.getSelected();
    	
    	// ===================
    	// setup logging to USB Stick (if one is plugged into one of the RoboRio USB Host ports)
    	// ===================
    	setupLogging("auton");

    }
    
    /*
     *****************************************************************************************************
     * This function is called periodically during autonomous mode
     * 	(about 50x / sec or about once every 20 mSec)
     * 
     * Every auton cycle calls thru and exist from this method
     * 	- we do a few general things 
     * 	- then this method will forward execution to the specific method that supports 
     *     the Auton Mode selected in autonomousInit
     *  - then this method updates all the outputs, optionally logs and updates the dashboard
     *****************************************************************************************************
     */
    public void autonomousPeriodic() 
    {
    	// increment loop counter
    	_autoLoopCounter++;
    
    	//get local references to make variable references shorter
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
    	
    	// ==========================
    	// Step 1: Get Inputs 
    	// ==========================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	
    	outputDataValues.DriversStationMsg = "";  
    	
    	// ==========================
    	// Step 2: call the appropriate auton mode
    	// ==========================
    	switch(inputDataValues.AutonModeRequested)
    	{
    		case DO_NOTHING:
    			autonomousUndefined();
    			break;
    			
    		case SIMPLE:
    			autonomousSimple();
    			break;
    			
    		case COMPLEX:
    			autonomousComplex();
    			break;
    	}
    	
    	// ==========================
    	// Step 3. Set Outputs
    	// 			In Auton Mode this is ONLY place where these should be set
    	// ==========================
    	
    	// make sure we only set the motor values in a real Auton Mode
    	if(inputDataValues.AutonModeRequested != AutonMode.DO_NOTHING)
    	{
	    	_liftMasterMtr.set(outputDataValues.LiftMtrAdjVelocityCmd);
	    	
	    	_leftInfeedMtr.set(outputDataValues.LeftInfeedMtrAdjVelocityCmd);
	    	_rightInfeedMtr.set(outputDataValues.RightInfeedMtrAdjVelocityCmd);
	    	
	    	_leftDriveMasterMtr.set(outputDataValues.ArcadeDriveThrottleAdjCmd);
	    	_rightDriveMasterMtr.set(outputDataValues.ArcadeDriveTurnAdjCmd);
	    	
	    	_canClamp.set(outputDataValues.CanClampPosition);
	    	_frontClips.set(outputDataValues.FrontClipPosition);
    	}
    	
    	// ==========================
    	// Step 4: Update the Dashboard
    	// ==========================
    	UpdateDashboard(_robotLiveData);
    	
    	// set last scan DT
    	_robotLiveData.WorkingDataValues.LastScanDT = new Date();
    	
    	// optionally send message to drivers station
    	if(_robotLiveData.OutputDataValues.DriversStationMsg != null 
    			&& _robotLiveData.OutputDataValues.DriversStationMsg.length() > 0)
    	{
    		DriverStation.reportError(_robotLiveData.OutputDataValues.DriversStationMsg, false);
    	}
    	
    	// ==========================
    	// 5.0 Optional Data logging
    	// ==========================
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    }
    
    /*
     ******************************************************
   	 * this is just a container for some old code
   	 ******************************************************
   	 */
    public void autonomousUndefined() 
    {
    	//toteLiftMasterMtr.set(0.1);
    	
    	_autoLoopCounter++;
    	
    	if (Math.abs(_robotLiveData.WorkingDataValues.LiftEncoderTotalDeltaCount) >= 2000)
    	{
    		_frontClips.set(RobotMap.FRONT_CLIPS_OPEN_POSITION);
    	}
    	
    	
    	
    	if(_autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			//myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			//autoLoopCounter++;
		} 
    	else 
    	{
			//myRobot.drive(0.0, 0.0); 	// stop robot
		}
    	
    	if((_autoLoopCounter >= 0) && (_autoLoopCounter < 100))
    	{
    		//_hitecServoMtr.setAngle(0);
    		_canClamp.set(DoubleSolenoid.Value.kForward);
    	}
    	if((_autoLoopCounter >= 100) && (_autoLoopCounter < 200))
    	{
    		//_hitecServoMtr.setAngle(45);
    		_canClamp.set(DoubleSolenoid.Value.kReverse);
    	}
    	if((_autoLoopCounter >= 200) && (_autoLoopCounter < 300))
    	{
    		//_hitecServoMtr.setAngle(90);
    		_frontClips.set(DoubleSolenoid.Value.kForward);
    		_canClamp.set(DoubleSolenoid.Value.kOff);
    	}
    	if((_autoLoopCounter >= 300) && (_autoLoopCounter < 400))
    	{
    		//_hitecServoMtr.setAngle(135);
    		_frontClips.set(DoubleSolenoid.Value.kReverse);
    	}
    	if((_autoLoopCounter >= 400) && (_autoLoopCounter < 500))
    	{
    		//_hitecServoMtr.setAngle(180);
    		_frontClips.set(DoubleSolenoid.Value.kOff);
    	}
    	//_liftEncoderCurrentCount = _liftMasterMtr.getPosition();
    	//toteLiftEncoderPosition = toteLiftMasterMtr.getEncPosition();
    	
    	//if (Math.abs(_liftEncoderCurrentCount - _liftEncoderInitialCount) >= 2000)
    	//{
    	//	_liftMasterMtr.set(0);
    	//}
    	//else 
    	//{
    	//	_liftMasterMtr.set(.05);
    	//}
    	
    	
    }
    
    /*
     ******************************************************
   	 * this is the default Auton Mode
   	 ******************************************************
   	 */
    public void autonomousDoNothing() 
    {
    }
    
    /*
     ******************************************************
   	 * simple test based on loop counter
   	 ******************************************************
   	 */
    public void autonomousSimple2() 
    {
    	if (_autoLoopCounter < 25)
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-01";
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_OPEN_POSITION;
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.0;
    	}
    	else if (_autoLoopCounter >=  25 && _autoLoopCounter < ((50 * 30) + 25)) 	// 50 loops / sec * 30 sec
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-02";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.25;
    	}
    	else
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-03";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.0;
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	}
    }
    
    /*
     ******************************************************
   	 * simple test based on dio value
   	 ******************************************************
   	 */
    public void autonomousSimple() 
    {
    	if (_robotLiveData.InputDataValues.IsToteInPosition != true)
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-01";
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	}
    	else
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-02";
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_OPEN_POSITION;
    	}
    }
    
    /*
     ******************************************************
   	 * more complex auton with velocity steps up and dow
   	 ******************************************************
   	 */
    public void autonomousComplex() 
    {
    	// 20 mSec = 1   scan
    	// 	5 secs = 250 scans
    	// 10 secs = 500 scans
    	final double mSecPerScan = 0.02;
    	final int secsPerStep = 5;		// <== adjust this value
    	int scanCntPerStep = (int)(mSecPerScan * secsPerStep);
    	
    	// step speed up then down to log accel, decel
    	if (_autoLoopCounter <= (1 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-01";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.10;
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_OPEN_POSITION;
    	}
    	else if (_autoLoopCounter <= (2 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-02";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.15;
    	}
    	else if (_autoLoopCounter <= (3 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-03";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.20;
    	}
    	else if (_autoLoopCounter <= (4 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-04";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.25;
    	}
    	else if (_autoLoopCounter <= (5 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-05";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.30;
    	}
    	else if (_autoLoopCounter <= (6 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-06";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.35;
    	}
    	else if (_autoLoopCounter <= (7 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-07";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.40;
    	}
    	else if (_autoLoopCounter <= (8 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-08";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.45;
    	}
    	else if (_autoLoopCounter <= (9 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-09";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.50;
    	}
    	else if (_autoLoopCounter <= (10 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-10";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.55;
    	}
    	else if (_autoLoopCounter <= (11 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-11";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.60;
    	}
    	else if (_autoLoopCounter <= (12 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-12";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.65;
    	}
    	else if (_autoLoopCounter <= (13 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-13";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.70;
    	}
    	else if (_autoLoopCounter <= (14 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-14";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.75;
    	}
    	else if (_autoLoopCounter <= (15 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-15";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.80;
    	}
    	else if (_autoLoopCounter <= (16 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-16";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.85;
    	}
    	else if (_autoLoopCounter <= (17 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-17";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.90;
    	}
    	else if (_autoLoopCounter <= (18 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-18";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.95;
    	}
    	else if (_autoLoopCounter <= (19 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-19";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 1.00;
    	}
    	// decel
    	else if (_autoLoopCounter <= (20 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-20";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.95;
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	}
    	else if (_autoLoopCounter <= (21 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-21";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.90;
    	}
    	else if (_autoLoopCounter <= (22 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-22";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.85;
    	}
    	else if (_autoLoopCounter <= (23 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-23";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.80;
    	}
    	else if (_autoLoopCounter <= (24 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-24";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.75;
    	}
    	else if (_autoLoopCounter <= (25 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-25";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.70;
    	}
    	else if (_autoLoopCounter <= (26 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-26";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.65;
    	}
    	else if (_autoLoopCounter <= (27 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-27";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.60;
    	}
    	else if (_autoLoopCounter <= (28 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-28";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.55;
    	}
    	else if (_autoLoopCounter <= (29 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-29";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.50;
    	}
    	else if (_autoLoopCounter <= (30 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-30";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.45;
    	}
    	else if (_autoLoopCounter <= (31 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-31";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.40;
    	}
    	else if (_autoLoopCounter <= (32 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-32";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.35;
    	}
    	else if (_autoLoopCounter <= (33 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-33";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.30;
    	}
    	else if (_autoLoopCounter <= (34 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-34";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.25;
    	}
    	else if (_autoLoopCounter <= (35 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-35";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.20;
    	}
    	else if (_autoLoopCounter <= (36 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-36";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.15;
    	}
    	else if (_autoLoopCounter <= (37 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-37";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.10;
    	}
    	else if (_autoLoopCounter <= (38 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-38";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.05;
    	}
    	else if (_autoLoopCounter > (38 * scanCntPerStep) )
    	{
    		_robotLiveData.WorkingDataValues.CurrentAutonState = "S-39";
    		_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.00;
    		_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_OPEN_POSITION;
    	}
    	
    	// set driver's station message on 1 loop in each step
    	if ((_autoLoopCounter % secsPerStep) == 1)
    	{
    		_robotLiveData.OutputDataValues.DriversStationMsg = "In Auton State: " + _robotLiveData.WorkingDataValues.CurrentAutonState;
    	}
    }
    
    /*
     *****************************************************************************************************
     * This function is called 1x each time the robot enters tele-operated mode
     *  (setup the initial robot state for telop mode)
     *****************************************************************************************************
     */
    public void teleopInit()
    {
    	_robotLiveData = new RobotData();
    	_robotLiveData.WorkingDataValues.CurrentAutonState = "N/A";
    	
    	// set a motors to 0 velocity command 
    	_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd = 0.0;
    	_robotLiveData.OutputDataValues.LeftInfeedMtrAdjVelocityCmd = 0.0;
    	_robotLiveData.OutputDataValues.RightInfeedMtrAdjVelocityCmd = 0.0;
    	_robotLiveData.OutputDataValues.ArcadeDriveThrottleAdjCmd = 0.0;
    	_robotLiveData.OutputDataValues.ArcadeDriveTurnAdjCmd = 0.0;
    	
    	_liftMasterMtr.set(_robotLiveData.OutputDataValues.LiftMtrAdjVelocityCmd);
    	
    	_leftInfeedMtr.set(_robotLiveData.OutputDataValues.LeftInfeedMtrAdjVelocityCmd);
    	_rightInfeedMtr.set(_robotLiveData.OutputDataValues.RightInfeedMtrAdjVelocityCmd);
    	
    	_leftDriveMasterMtr.set(_robotLiveData.OutputDataValues.ArcadeDriveThrottleAdjCmd);
    	_rightDriveMasterMtr.set(_robotLiveData.OutputDataValues.ArcadeDriveTurnAdjCmd);
    	
    	// init the drive speed scaling factor to full speed
    	_robotLiveData.WorkingDataValues.DriveSpeedScalingFactor = 1.0;
    	_robotLiveData.WorkingDataValues.IsDriveSpeedScalingButtonPressedLastScan = false;
    	
    	// init the values that control Lift Position Holding
    	_robotLiveData.WorkingDataValues.IsLiftHoldPositionButtonPressedLastScan = false;
    	_robotLiveData.WorkingDataValues.IsLiftHoldPositionModeEnabled  = false;
    	_robotLiveData.WorkingDataValues.IsLiftHoldPositionModeActive = false;
    	    	
    	// enable limit switches on tote lift axis (in case they were previously manually overridden)
    	_liftMasterMtr.enableLimitSwitch(true, true);
    	
    	// turn off rumble (only works on XBox Gamepad Controllers)
    	_operatorGamepad.setRumble(RumbleType.kLeftRumble, 0); 
    	
    	// initialize axis (Encoder) positions
    	_liftMasterMtr.setPosition(0);			// TODO: this should really only happen at home position
    	_leftDriveMasterMtr.setPosition(0);
    	_rightDriveMasterMtr.setPosition(0);
    	
    	// get initial values from Encoders
    	_robotLiveData.WorkingDataValues.LiftEncoderInitialCount = _liftMasterMtr.getPosition();
    	_robotLiveData.WorkingDataValues.LeftDriveEncoderInitialCount = _leftDriveMasterMtr.getPosition();
    	_robotLiveData.WorkingDataValues.RightDriveEncoderInitialCount = _rightDriveMasterMtr.getPosition();
    	
    	// get values from Smart Dashboard (this is not really needed for Telop mode)
    	_robotLiveData.InputDataValues.AutonModeRequested = (AutonMode) autonModeChooser.getSelected();
    	
    	// set our desired default state for the can clamp and front clips
    	_robotLiveData.OutputDataValues.CanClampPosition = RobotMap.CAN_CLAMP_OPEN_POSITION;
    	_robotLiveData.OutputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	
    	// set the inital states to solenoids
    	_canClamp.set(_robotLiveData.OutputDataValues.CanClampPosition);
    	_frontClips.set(_robotLiveData.OutputDataValues.FrontClipPosition);
    	
    	// ===================
    	// optionally setup logging to USB Stick (if it is plugged into one of the RoboRio Host USB ports)
    	// ===================
    	setupLogging("telop");
    }
    	
    /*
     *****************************************************************************************************
     * This function is called periodically during operator control
     * 	(about 50x/sec or about every 20 mSec)
     * 
     * Four (4) main steps
     * 	1.	Get Inputs
     *  2. 	Update Working Values
     *  3.	Calc new motor speeds
     *  4.	Set Outputs
     *  5.	Update Dashboard
     * 
     *****************************************************************************************************
     */
    public void teleopPeriodic() 
    {
    	// =====================================
    	// === Step 0: get local references for objects that are properties of _robotLiveData to
    	//				make variable references shorter
    	// =====================================
    	InputData inputDataValues = _robotLiveData.InputDataValues;
    	WorkingData workingDataValues = _robotLiveData.WorkingDataValues;
    	OutputData outputDataValues = _robotLiveData.OutputDataValues;
        	
    	// =====================================
    	// Step 1: Get Inputs  and Update Working Values
    	// =====================================
    	UpdateInputAndCalcWorkingDataValues(inputDataValues, workingDataValues);
    	outputDataValues.DriversStationMsg = "";    
    	
    	// =====================================
    	// === Step 2: Calc New Motor Speeds ===
    	// =====================================
    	
    	// In Telop Mode this is ONLY place where the motor speeds should be calculated
    	
    	// ==========================
    	// 2.1 Drive Motor speed
    	// ==========================
    	// set the drive speed scale factor (currently we support .25, .5, .75 & 1.0)
    	// 	notes: 	this is a toggle,  the previous value is retained between scans
    	//			need to de-bounce key press since the scan rate is so fast 
    	if((inputDataValues.IsScaleDriveSpeedUpBtnPressed == true) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed == true))
    	{
    		// Don't change scale factor if both buttons are pressed
    	}
    	else if (((inputDataValues.IsScaleDriveSpeedUpBtnPressed == true) 
    						|| (inputDataValues.IsScaleDriveSpeedDownBtnPressed == true))
    			&& (workingDataValues.IsDriveSpeedScalingButtonPressedLastScan == true))
    	{
    		// Don't change scale factor if either button pressed now and on the last scan, ie. de-bounce
    	}
    	else if((inputDataValues.IsScaleDriveSpeedUpBtnPressed == true ) 
    			&& (workingDataValues.DriveSpeedScalingFactor < 1.0))
    	{
    		// scale up
    		workingDataValues.DriveSpeedScalingFactor 
    				= workingDataValues.DriveSpeedScalingFactor + .25;
    	}
    	else if((inputDataValues.IsScaleDriveSpeedDownBtnPressed == true) 
    			&& (workingDataValues.DriveSpeedScalingFactor > 0.25))
    	{
    		// scale down
    		workingDataValues.DriveSpeedScalingFactor 
    			= workingDataValues.DriveSpeedScalingFactor - .25;
    	}
    	else if((inputDataValues.IsScaleDriveSpeedUpBtnPressed != true) 
    			&& (inputDataValues.IsScaleDriveSpeedDownBtnPressed != true))
    	{
    		// if neither button is pressed do nothing
    	}
    	
    	outputDataValues.ArcadeDriveThrottleAdjCmd 
    			= inputDataValues.ArcadeDriveThrottleRawCmd; // * workingDataValues.DriveSpeedScalingFactor;
    	
    	outputDataValues.ArcadeDriveTurnAdjCmd 
    			= inputDataValues.ArcadeDriveTurnRawCmd * -0.2; //workingDataValues.DriveSpeedScalingFactor*-0.2; flip for mtr orientation

    	// ==========================
    	// 2.2 Lift Motor speed
    	// ==========================
		// by default use any lift speed command from the operator gamepads
		outputDataValues.LiftMtrAdjVelocityCmd = inputDataValues.LiftMtrRawVelocityCmd;
		
		// IsLiftHoldPositionModeEnabled: 	true if the operator wants us to hold the lift position
		// IsLiftHoldPositionModeActive:	true if we should actively be trying to hold the lift position
		
		// toggle the IsLiftHoldPositionModeEnabled based on leading edge of button state change
		if ((inputDataValues.IsLiftHoldPositionModeBtnPressed == true) 
				&& (workingDataValues.IsLiftHoldPositionButtonPressedLastScan == false))
		{
			workingDataValues.IsLiftHoldPositionModeEnabled 
				= ! workingDataValues.IsLiftHoldPositionModeEnabled;
		}
		else if ((inputDataValues.IsLiftHoldPositionModeBtnPressed == true) 
				&& (workingDataValues.IsLiftHoldPositionButtonPressedLastScan == true))
		{
			// Don't change state if button pressed now and on the last scan, ie. de-bounce
		}
				
		// if the current lift speed command from the operator is 0
		//	and the last lift speed command from the operator is <> 0  (this means they just stopped driving the axis)
		//  and the hold lift position mode is active
    	if (inputDataValues.LiftMtrRawVelocityCmd == 0
    			&& workingDataValues.LiftMtrLastScanRawVelocityCmd != 0
    			&& workingDataValues.IsLiftHoldPositionModeEnabled == true)
    	{
    		// snapshot the target elevator position and set the mode to active
    		workingDataValues.LiftEncoderHoldTargetCount = inputDataValues.LiftEncoderCurrentCount;
    		workingDataValues.IsLiftHoldPositionModeActive = true;
    	}
    	else if (inputDataValues.LiftMtrRawVelocityCmd != 0)
    	{
    		// if there is any input command set the mode to inactive
    		workingDataValues.IsLiftHoldPositionModeActive = false;
    	}
    	
    	// implement hold lift position mode
    	if(workingDataValues.IsLiftHoldPositionModeActive == true)
    	{
    		// calc the error in the position, (+) value = actual BELOW target
    		double liftPositionErrorCount = workingDataValues.LiftEncoderHoldTargetCount 
    											- inputDataValues.LiftEncoderCurrentCount;    		
    		double liftPositionErrorInches = liftPositionErrorCount * RobotMap.LIFT_AXIS_TRAVEL_DISTANCE_INCHES_PER_COUNT;
    		
    		// axis should only drift down, so drive back up using very crude gain scheduling / prop control
    		if(liftPositionErrorInches <= 0.1)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0;		// 0% if <= .1"
    		}
    		else if(liftPositionErrorInches <= 0.15)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0.07;	// 7% if > .1" and <= .15"
    		}
    		else if(liftPositionErrorInches <= 0.25)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0.1;	// 10% if > .15" and <= .25"
    		}
    		else if(liftPositionErrorInches <= 0.5)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0.2;	// 20% if > .25" and <= .5"
    		}
    		else if(liftPositionErrorInches <= 1.0)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0.3;	// 30% if > .5" and <= 1.0"
    		}
    		else if(liftPositionErrorInches > 1.0)
    		{
    			outputDataValues.LiftMtrAdjVelocityCmd = 0.4;	// 40% if > 1.0"
    		}
    	}

    	// ==========================
    	// 2.3 Infeed Motor speed
    	// ==========================
    	
    	/*
    	 * Button Mapping for Infeed Motors
    	 * 	
    	 * 		L Trigger - rotate left wheel clockwise (trigger allows for variable speed control)
    	 * 		R Trigger - rotate right wheel counterclockwise
    	 * 		L Button - Infeed 
    	 * 		R Button - Outfeed
    	 * 
    	 */
    	
    	// these will calc the final commands sent to the motor controllers
    	
    	// Old code for running infeed wheels using right operator joystick
    	/*
    	double joystickDeadband = 0.1;
    	
    	// allow operator to pull/push straight in or out by driving infeed wheels both in or out (using Y Axis Joystick)
    	if((inputDataValues.InfeedMtrRawInOutVelocityCmd < (-1.0 * joystickDeadband)) || (inputDataValues.InfeedMtrRawInOutVelocityCmd > joystickDeadband))	// use a deadband
    	{
    		outputDataValues.LeftInfeedMtrAdjVelocityCmd = inputDataValues.InfeedMtrRawInOutVelocityCmd * -1.0;	// flip for mtr orientation    
    		outputDataValues.RightInfeedMtrAdjVelocityCmd = inputDataValues.InfeedMtrRawInOutVelocityCmd;
    	}
    	// allow operator to "spin" by driving infeed wheels one in, the other out (using X Axis Joystick)
    	else if((inputDataValues.InfeedMtrRawSpinVelocityCmd < (-1.0 * joystickDeadband)) || (inputDataValues.InfeedMtrRawSpinVelocityCmd > joystickDeadband))	// use a deadband
    	{
    		outputDataValues.LeftInfeedMtrAdjVelocityCmd = inputDataValues.InfeedMtrRawSpinVelocityCmd;
    		outputDataValues.RightInfeedMtrAdjVelocityCmd = inputDataValues.InfeedMtrRawSpinVelocityCmd;
    	}
    	// do not drive infeed wheels
    	else
    	{
    		outputDataValues.LeftInfeedMtrAdjVelocityCmd = 0.0;
    		outputDataValues.RightInfeedMtrAdjVelocityCmd = 0.0;
    	}
    	*/		
    	if ((inputDataValues.IsInfeedInBtnPressed == true) 
				&& (inputDataValues.IsInfeedOutBtnPressed == false))
		{
    		_robotLiveData.OutputDataValues.LeftInfeedMtrAdjVelocityCmd = 1.0;
        	_robotLiveData.OutputDataValues.RightInfeedMtrAdjVelocityCmd = -1.0;
		}
    	else if ((inputDataValues.IsInfeedInBtnPressed == false) 
				&& (inputDataValues.IsInfeedOutBtnPressed == true))
		{
    		_robotLiveData.OutputDataValues.LeftInfeedMtrAdjVelocityCmd = -1.0;
        	_robotLiveData.OutputDataValues.RightInfeedMtrAdjVelocityCmd = 1.0;
		}
    	else
    	{
    		//Don't run infeed wheels if both buttons are pressed
    	}
    	_robotLiveData.OutputDataValues.LeftInfeedMtrAdjVelocityCmd = inputDataValues.LeftInfeedMtrRawVelocityCmd;
    	_robotLiveData.OutputDataValues.RightInfeedMtrAdjVelocityCmd = inputDataValues.RightInfeedMtrRawVelocityCmd;
    	//Sets infeed speed based on values read from trigger
    	//NOTE: No code yet to prevent conflict between the buttons and triggers being pressed simeltaneously, feature needs to be added
    	
    	// =====================================
    	// ======= Step 3: Set Outputs =========
    	// =====================================
    		
    	// ==========================
    	// 3.1 Handle front tote clips
    	//		Solenoids work like a toggle, the current value is retained until it is changed
    	// ==========================
    	if ((inputDataValues.IsClipOpenBtnPressed == true) && (inputDataValues.IsClipClosedBtnPressed == true))
    	{
    		// Don't change clip position if both buttons are pressed
    	}
    	else if (inputDataValues.IsClipOpenBtnPressed == true)
    	{
    		outputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_OPEN_POSITION;
    	}
    	else if (inputDataValues.IsClipClosedBtnPressed == true)
    	{
    		outputDataValues.FrontClipPosition = RobotMap.FRONT_CLIPS_CLOSED_POSITION;
    	}
    	
    	// ==========================
    	// 3.2 Handle Can Clamp
    	//		Solenoids work like a toggle, the current value is retained until it is changed
    	// ==========================
    	if ((inputDataValues.IsCanClampBtnPressed == true) && (inputDataValues.IsCanUnclampBtnPressed == true))
    	{
    		// Don't change clamp position if both buttons are pressed
    	}
    	else if (inputDataValues.IsCanClampBtnPressed == true)
    	{
    		outputDataValues.CanClampPosition = RobotMap.CAN_CLAMP_CLOSED_POSITION;
    	}
    	else if (inputDataValues.IsCanUnclampBtnPressed == true)
    	{
    		outputDataValues.CanClampPosition = RobotMap.CAN_CLAMP_OPEN_POSITION;
    	}
    	
    	// ==========================
    	// 3.3 Handle Lift Override Limit Switch Button
    	//		This is NOT a toggle, the operator must continue to press the button
    	// ==========================
    	if (inputDataValues.IsLiftLimitSwitchOverrideBtnPressed == true)
    	{
    		_liftMasterMtr.enableLimitSwitch(false, false);
    	}
    	else 
    	{
    		_liftMasterMtr.enableLimitSwitch(true, true);
    	}
    	
    	// ==========================
    	// 3.4 set rumble on lift drive (operator's) gamepad while drive cmd > 0 AND already on axis eot limit switch
    	// 		NOTE: Logitech Gamepad F310 controllers DO NOT support rumble	
    	// ==========================
    	if ((inputDataValues.IsLiftAtTopEOT == true) && (outputDataValues.LiftMtrAdjVelocityCmd > 0.0))
    	{
    		// warn operator we are at the top eot (on fwd limit switch)
    		_operatorGamepad.setRumble(RumbleType.kLeftRumble, 1);
    	}
    	else if ((inputDataValues.IsLiftAtBottomEOT) && (outputDataValues.LiftMtrAdjVelocityCmd < 0.0))
    	{
    		// warn operator we are at the bottom (on rev eot limit switch)
    		_operatorGamepad.setRumble(RumbleType.kLeftRumble, 1);
    	}
    	else
    	{
    		// turn off rumble
    		_operatorGamepad.setRumble(RumbleType.kLeftRumble, 0);
    	}
    	
    	// ==========================
    	// 3.5 Set Motor & Solenoid Outputs
    	// ==========================
    	// In Telop Mode this is ONLY place where these should be set
    	_liftMasterMtr.set(outputDataValues.LiftMtrAdjVelocityCmd);
    	
    	_leftInfeedMtr.set(outputDataValues.LeftInfeedMtrAdjVelocityCmd);
    	_rightInfeedMtr.set(outputDataValues.RightInfeedMtrAdjVelocityCmd);
    	
    	_robotDrive.arcadeDrive(outputDataValues.ArcadeDriveThrottleAdjCmd, outputDataValues.ArcadeDriveTurnAdjCmd, true);
    	
    	_canClamp.set(outputDataValues.CanClampPosition);
    	_frontClips.set(outputDataValues.FrontClipPosition);
    	
    	// ==========================
    	// 4.0 Update Dashboard
    	// ==========================
    	UpdateDashboard(_robotLiveData);
    	
    	_robotLiveData.WorkingDataValues.LastScanDT = new Date();  	
    	
    	// optionally send message to drivers station
    	if(_robotLiveData.OutputDataValues.DriversStationMsg != null 
    			&& _robotLiveData.OutputDataValues.DriversStationMsg.length() > 0)
    	{
    		DriverStation.reportError(_robotLiveData.OutputDataValues.DriversStationMsg, false);
    	}
    	
    	// ==========================
    	// 5.0 Optional Data logging
    	// ==========================
    	if(workingDataValues.IsLoggingEnabled == true)
    	{
    		_dataLogger.WriteData(_robotLiveData);
    	}
    	// set last scan DT

    	// ==========================
    	// 6.0 Stuff we want to do at the very end
    	// ==========================
    	workingDataValues.IsLiftHoldPositionButtonPressedLastScan 
    						= inputDataValues.IsLiftHoldPositionModeBtnPressed;
    	
    	workingDataValues.IsDriveSpeedScalingButtonPressedLastScan 
    						= (inputDataValues.IsScaleDriveSpeedUpBtnPressed
    								|| inputDataValues.IsScaleDriveSpeedDownBtnPressed);
    }
    
    /**
    / This method updates all of the input values
	**/
    private void UpdateInputAndCalcWorkingDataValues(InputData inputDataValues, WorkingData workingDataValues)
    {	
    	// ==========================
    	// 1.1 get hign resolution timer
    	// ==========================
    	inputDataValues.FPGATimeMicroSecs = Utility.getFPGATime();
    	
    	// ==========================
    	// 1.2 get values from the gamepads
    	// ==========================
    	inputDataValues.IsCanClampBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CAN_CLAMP_BTN);
    	inputDataValues.IsCanUnclampBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_CAN_UNCLAMP_BTN);
    	inputDataValues.IsLiftLimitSwitchOverrideBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_LIFT_LIMIT_SWITCH_OVERRIDE_BTN);
    	inputDataValues.IsLiftHoldPositionModeBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_LIFT_HOLD_POSITION_MODE_BTN);
    	
    	inputDataValues.IsClipOpenBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CLIP_OPEN_BTN);
    	inputDataValues.IsClipClosedBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_CLIP_CLOSE_BTN);
    	inputDataValues.IsScaleDriveSpeedUpBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN);
    	inputDataValues.IsScaleDriveSpeedDownBtnPressed = _driverGamepad.getRawButton(RobotMap.DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN);
    	inputDataValues.IsInfeedInBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_IN_BUTTON);
    	inputDataValues.IsInfeedOutBtnPressed = _operatorGamepad.getRawButton(RobotMap.OPERATOR_GAMEPAD_INFEED_OUT_BUTTON);
    	
    	// remember:	on gamepads fwd/up = -1 and rev/down = +1 so invert the values
    	//				this is to logically correct the way gamepads works so that fwd/up is +
    	//				some values may need to inverted back later if CCW motor direction = FWD/UP instead of CW
    	inputDataValues.LiftMtrRawVelocityCmd = (-1.0) * _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_LIFT_AXIS_JOYSTICK);
    	inputDataValues.LeftInfeedMtrRawVelocityCmd = (-1.0) * _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_INFEED_ROTATE_L_TRIGGER);
    	inputDataValues.RightInfeedMtrRawVelocityCmd = (-1.0) * _operatorGamepad.getRawAxis(RobotMap.OPERATOR_GAMEPAD_INFEED_ROTATE_R_TRIGGER);
    	inputDataValues.ArcadeDriveThrottleRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK);
    	inputDataValues.ArcadeDriveTurnRawCmd = _driverGamepad.getRawAxis(RobotMap.DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK);
    	    	
    	// ==========================
    	// 1.3 get values from Lift Axis Limit Switches
    	//		(invert values since we are using normally closed limit switches)
    	// ==========================
    	inputDataValues.IsLiftAtTopEOT = !_liftMasterMtr.isFwdLimitSwitchClosed();
    	inputDataValues.IsLiftAtBottomEOT = !_liftMasterMtr.isRevLimitSwitchClosed();
    	
    	// ==========================
    	// 1.4 get values from axis position Encoders
    	// ==========================
    	inputDataValues.LiftEncoderCurrentCount = _liftMasterMtr.getPosition();
    	inputDataValues.LeftDriveEncoderCurrentCount = _leftDriveMasterMtr.getPosition();
    	inputDataValues.RightDriveEncoderCurrentCount = _rightDriveMasterMtr.getPosition();
    	    	
    	// ==========================
    	// 1.5 get values from DIO
    	// ==========================
    	inputDataValues.IsToteInPosition = !(_toteInPositionSensor.get());	// invert since this is a N/O sensor and the DIO input is pulled high    	
				 
    	// ==========================
    	// 1.5 get values from navX
    	// ==========================
    	inputDataValues.NavxIsConnected = _navXSensor.isConnected();
    	inputDataValues.NavxIsCalibrating = _navXSensor.isCalibrating();
    	inputDataValues.NavxYaw = _navXSensor.getYaw();
    	inputDataValues.NavxPitch = _navXSensor.getPitch();
    	inputDataValues.NavxRoll = _navXSensor.getRoll();
    	inputDataValues.NavxCompassHeading = _navXSensor.getCompassHeading();
    	inputDataValues.NavxFusedHeading = _navXSensor.getFusedHeading();
    	inputDataValues.NavxTotalYaw = _navXSensor.getAngle();
    	inputDataValues.NavxYawRateDPS = _navXSensor.getRate();
    	inputDataValues.NavxAccelX = _navXSensor.getWorldLinearAccelX();
    	inputDataValues.NavxAccelY = _navXSensor.getWorldLinearAccelY();
    	inputDataValues.NavxIsMoving = _navXSensor.isMoving();
    	inputDataValues.NavxIsRotating = _navXSensor.isRotating();
    	
    	// =========================
    	// 2.0 Calc Working Values
    	// ==========================
    	
    	// speed units are are sensor's native ticks per 100mSec
    	//  1000 counts => 10 RPS (Rotation per second)
    	
    	// 2.1 Lift Axis
		workingDataValues.LiftEncoderLastDeltaCount = (inputDataValues.LiftEncoderCurrentCount 
														- workingDataValues.LiftEncoderLastCount);
		workingDataValues.LiftEncoderTotalDeltaCount = (inputDataValues.LiftEncoderCurrentCount 
														- workingDataValues.LiftEncoderInitialCount);
		workingDataValues.LiftEncoderLastCount = inputDataValues.LiftEncoderCurrentCount;
    	
    	workingDataValues.LiftAxisEncoderCurrentCPS = _liftMasterMtr.getSpeed() * 10.0;

    	workingDataValues.LiftAxisCurrentSpeedIPS = workingDataValues.LiftAxisEncoderCurrentCPS
    													* RobotMap.LIFT_AXIS_TRAVEL_DISTANCE_INCHES_PER_COUNT;

    	workingDataValues.LiftAxisGearBoxCurrentRPM = (workingDataValues.LiftAxisEncoderCurrentCPS 
														* 60		// CPS -> CPM
														/ RobotMap.LIFT_AXIS_ENCODER_COUNTS_PER_REV);		// CPM -> RPM

    	workingDataValues.LiftAxisMotorCurrentRPM = workingDataValues.LiftAxisGearBoxCurrentRPM		
														* RobotMap.LIFT_AXIS_GEAR_BOX_RATIO;
    	
    	// 2.2 Left Axis
		workingDataValues.LeftDriveEncoderLastDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderLastCount);
		workingDataValues.LeftDriveEncoderTotalDeltaCount = (inputDataValues.LeftDriveEncoderCurrentCount 
																- workingDataValues.LeftDriveEncoderInitialCount);
		workingDataValues.LeftDriveEncoderLastCount = inputDataValues.LeftDriveEncoderCurrentCount;
    	
    	workingDataValues.LeftDriveEncoderCurrentCPS = _leftDriveMasterMtr.getSpeed() * 10.0;

    	workingDataValues.LeftDriveWheelsCurrentSpeedIPS = workingDataValues.LeftDriveEncoderCurrentCPS
    														* RobotMap.LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;

    	workingDataValues.LeftDriveGearBoxCurrentRPM = (workingDataValues.LeftDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.LEFT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM

		workingDataValues.LeftDriveMotorCurrentRPM = workingDataValues.LeftDriveGearBoxCurrentRPM
														* RobotMap.LEFT_DRIVE_GEAR_BOX_RATIO;
    	
		// 2.3 Right Axis
		workingDataValues.RightDriveEncoderLastDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderLastCount);
		workingDataValues.RightDriveEncoderTotalDeltaCount = (inputDataValues.RightDriveEncoderCurrentCount 
																- workingDataValues.RightDriveEncoderInitialCount);
		workingDataValues.RightDriveEncoderLastCount = inputDataValues.RightDriveEncoderCurrentCount;
		
    	workingDataValues.RightDriveEncoderCurrentCPS = _rightDriveMasterMtr.getSpeed() * 10.0;
    	
    	workingDataValues.RightDriveWheelsCurrentSpeedIPS = workingDataValues.RightDriveEncoderCurrentCPS
    															* RobotMap.RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT;
    	
    	workingDataValues.RightDriveGearBoxCurrentRPM = (workingDataValues.RightDriveEncoderCurrentCPS 
															* 60										// CPS -> CPM
															/ RobotMap.RIGHT_DRIVE_ENCODER_COUNTS_PER_REV);		// CPM -> RPM
    	
    	workingDataValues.RightDriveMotorCurrentRPM = workingDataValues.RightDriveGearBoxCurrentRPM
														* RobotMap.RIGHT_DRIVE_GEAR_BOX_RATIO;
    }
    
    /**
    / This method updates the Smart Dashboard
	**/
    private void UpdateDashboard(RobotData robotDataValues)
    {
    	//get local references
    	InputData inputDataValues = robotDataValues.InputDataValues;
    	WorkingData workingDataValues = robotDataValues.WorkingDataValues;
    	OutputData outputDataValues = robotDataValues.OutputDataValues;
    	
    	// Smart Dashboard Input
    	SmartDashboard.putString("SD:AutonMode", inputDataValues.AutonModeRequested.toString());
    	
    	SmartDashboard.putString("SD:AutonState", workingDataValues.CurrentAutonState);
    	
    	// Lift Motor
    	SmartDashboard.putBoolean("Lift.LS:IsAtTopEOT", inputDataValues.IsLiftAtTopEOT);
    	SmartDashboard.putBoolean("Lift.LS:IsAtBottomEOT", inputDataValues.IsLiftAtBottomEOT);
    	
		SmartDashboard.putNumber("Lift.Joy:RawVelCmd", inputDataValues.LiftMtrRawVelocityCmd);
		SmartDashboard.putNumber("Lift.Mtr:AdjVelCmd", outputDataValues.LiftMtrAdjVelocityCmd);
		
		SmartDashboard.putNumber("Lift.Enc:CurSpeedCPS", workingDataValues.LiftAxisEncoderCurrentCPS);
		SmartDashboard.putNumber("Lift.Axis:CurSpeedIPS", workingDataValues.LiftAxisCurrentSpeedIPS);
		SmartDashboard.putNumber("Lift.Mtr:CurSpeedRPM", workingDataValues.LiftAxisMotorCurrentRPM);
		SmartDashboard.putNumber("Lift.GB:CurSpeedRPM", workingDataValues.LiftAxisGearBoxCurrentRPM);
		
		SmartDashboard.putNumber("Lift.Enc:InitCount", workingDataValues.LiftEncoderInitialCount);
		SmartDashboard.putNumber("Lift.Enc:CurrCount", robotDataValues.InputDataValues.LiftEncoderCurrentCount);
		SmartDashboard.putNumber("Lift.Enc:DeltaCount", workingDataValues.LiftEncoderTotalDeltaCount);

		// Infeed Motor
		SmartDashboard.putNumber("Infeed.Trigger:RawLeftVelCmd", inputDataValues.LeftInfeedMtrRawVelocityCmd);
		SmartDashboard.putNumber("Infeed.Trigger:RawRightVelCmd", inputDataValues.RightInfeedMtrRawVelocityCmd);
		
		SmartDashboard.putNumber("Infeed.Mtr:LeftAdjVelCmd", outputDataValues.LeftInfeedMtrAdjVelocityCmd);
		SmartDashboard.putNumber("Infeed.Mtr:RightAdjVelCmd", outputDataValues.RightInfeedMtrAdjVelocityCmd);		 
				
		// Drive Motors
		SmartDashboard.putNumber("Drive.Btn:SpeedScaleFactor", workingDataValues.DriveSpeedScalingFactor);
		
		SmartDashboard.putNumber("Drive.Left:JoyThrottleRawCmd", inputDataValues.ArcadeDriveThrottleRawCmd);
		SmartDashboard.putNumber("Drive.Right:JoyTurnRawCmd", inputDataValues.ArcadeDriveTurnRawCmd);
				
		SmartDashboard.putNumber("Drive.Left:ArcadeDriveThrottleCmd", outputDataValues.ArcadeDriveThrottleAdjCmd);
		SmartDashboard.putNumber("Drive.Right:ArcadeDriveTurnCmd", outputDataValues.ArcadeDriveTurnAdjCmd);
		
		SmartDashboard.putNumber("Drive.Left:EncInitCount", workingDataValues.LeftDriveEncoderInitialCount);
		SmartDashboard.putNumber("Drive.Left:EncCurrCount", inputDataValues.LeftDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Left:EncDeltaCount", workingDataValues.LeftDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Left:MtrCurSpeedRPM", workingDataValues.LeftDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:GBCurSpeedRPM", workingDataValues.LeftDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Left:EncCurSpeedCPS", workingDataValues.LeftDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Left:WheelCurSpeedIPS", workingDataValues.LeftDriveWheelsCurrentSpeedIPS);

		SmartDashboard.putNumber("Drive.Right:EncInitCount", workingDataValues.RightDriveEncoderInitialCount);  
		SmartDashboard.putNumber("Drive.Right:EncCurCount", inputDataValues.RightDriveEncoderCurrentCount);
		SmartDashboard.putNumber("Drive.Right:EncDeltaCount", workingDataValues.RightDriveEncoderTotalDeltaCount);
		
		SmartDashboard.putNumber("Drive.Right:MtrCurSpeedRPM", workingDataValues.RightDriveMotorCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:GBCurSpeedRPM", workingDataValues.RightDriveGearBoxCurrentRPM);
		SmartDashboard.putNumber("Drive.Right:EncCurSpeedCPS", workingDataValues.RightDriveEncoderCurrentCPS);
		SmartDashboard.putNumber("Drive.Right:WheelCurSpeedIPS", workingDataValues.RightDriveWheelsCurrentSpeedIPS);

		// DIO
		SmartDashboard.putBoolean("DIO:IsToteInPosition", inputDataValues.IsToteInPosition); 
		
		// Logging
		SmartDashboard.putBoolean("Log:IsLoggingEnabled", workingDataValues.IsLoggingEnabled);
		SmartDashboard.putString("Log:LogFilePathName", workingDataValues.LogFilePathName); 
		
		// Navigation Sensor
		SmartDashboard.putBoolean("NavX_IsConnected", inputDataValues.NavxIsConnected);
        SmartDashboard.putBoolean("NavX_IsCalibrating", inputDataValues.NavxIsCalibrating);
        SmartDashboard.putNumber("NavX_Yaw", inputDataValues.NavxYaw);
        SmartDashboard.putNumber("NavX_Pitch", inputDataValues.NavxPitch);
        SmartDashboard.putNumber("NavX_Roll", inputDataValues.NavxRoll);
        SmartDashboard.putNumber("NavX_CompassHeading", inputDataValues.NavxCompassHeading);
        SmartDashboard.putNumber("NavX_FusedHeading", inputDataValues.NavxFusedHeading); 
        SmartDashboard.putNumber("NavX_TotalYaw", inputDataValues.NavxTotalYaw); 
        SmartDashboard.putNumber("NavX_YawRateDPS", inputDataValues.NavxYawRateDPS); 
        SmartDashboard.putNumber("NavX_Accel_X", inputDataValues.NavxAccelX); 
        SmartDashboard.putNumber("NavX_Accel_Y", inputDataValues.NavxAccelY); 
        SmartDashboard.putBoolean("NavX_IsMoving", inputDataValues.NavxIsMoving); 
        SmartDashboard.putBoolean("NavX_IsRotating", inputDataValues.NavxIsRotating); 
        
        SmartDashboard.putString("Misc:LastUpdateDT", ZonedDateTime.now().toString());
    }
    
    /**
    / This method optionally sets up logging
	**/
	private void setupLogging(String mode) 
	{
		// see if the USB stick is plugged into to RoboRIO
		Path path = Paths.get(RobotMap.LOG_FILE_PATH);
    	if (Files.exists(path)) 
    	{
    		try 
    		{
				_dataLogger = new DataLogger(RobotMap.LOG_FILE_PATH, mode);
				_dataLogger.WriteData(_robotLiveData.BuildTSVHeader());
				
				_robotLiveData.WorkingDataValues.LogFilePathName = _dataLogger.LogFilePathName;
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = true;
	    		_robotLiveData.WorkingDataValues.LoggingStartedDT = new Date();
	    		_robotLiveData.WorkingDataValues.LastScanDT = new Date();
			} 
    		catch (IOException e) 
    		{
				// TODO Auto-generated catch block
				e.printStackTrace();
				
	    		_dataLogger = null;
				_robotLiveData.WorkingDataValues.LogFilePathName = "";
	    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
			}
    	}
    	else
    	{
    		_dataLogger = null;
			_robotLiveData.WorkingDataValues.LogFilePathName = "";
    		_robotLiveData.WorkingDataValues.IsLoggingEnabled = false;
    	}
	}
	
	public void testInit()
	{	
	}
	
    /*****************************************************************************************************
     * This function is called periodically during test mode
     * 	(about 50x/sec or about every 20 mSec)
     *****************************************************************************************************/
    public void testPeriodic() 
    {
    	// this mode is not currently implemented 
    	
    	//LiveWindow.run();
    }
    
}
