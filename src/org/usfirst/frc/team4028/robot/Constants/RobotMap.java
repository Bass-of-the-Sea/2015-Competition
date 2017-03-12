package org.usfirst.frc.team4028.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class contains global constants that define the layout of Team 4028's 2015 Season Recycle Rush Robot
 * 
 * 
 * Date			Rev		Author						Comments
 * -----------	------	-------------------------	---------------------------------- 
 * 23.Aug.2015	0.1		Tom Bruns					Refactored and moved constants to a new class
 * 27.Jun.2015	0.1		Sebastian Rodriguez			Initial Version
 */
public class RobotMap 
{	
	// ======================================
	// Constants for CAN Bus Addresses
	// ======================================
	
	// define constant for PCM (Pneumatic Control Module)
	public static final int CAN_ADDR_PCM = 4;				
	
	// define constants for Talon SRX CAN Bus Addresses
	public static final int CAN_ADDR_LIFT_MASTER_MTR = 11;
	public static final int CAN_ADDR_LIFT_SLAVE_MTR = 10;
	public static final int CAN_ADDR_LEFT_DRIVE_MASTER_MTR = 14;
	public static final int CAN_ADDR_LEFT_DRIVE_SLAVE_MTR = 16;
	public static final int CAN_ADDR_RIGHT_DRIVE_MASTER_MTR = 15;
	public static final int CAN_ADDR_RIGHT_DRIVE_SLAVE_MTR = 17;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	public static final int PWM_PORT_LEFT_INFEED_MOTOR = 2;
	public static final int PWM_PORT_RIGHT_INFEED_MOTOR = 3;
	//public static final int PWM_PORT_TEST_MOTOR = 6;
	//public static final int PWM_PORT_HITEC_SERVO_MOTOR = 9;
	
	// ======================================
	// define constants for Encoder Feedback
	// ======================================
	public static final double LEFT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1
	public static final int LEFT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;		// 6" Wheel Dia, C = 2*Pi*R
	
	public static final double RIGHT_DRIVE_GEAR_BOX_RATIO = 14.88;						// 14:88 : 1
	public static final int RIGHT_DRIVE_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)
	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV = 18.850;	// 6" Wheel Dia, C = 2*Pi*R

	public static final double LIFT_AXIS_GEAR_BOX_RATIO = 14.88;						// 24 : 1
	public static final int LIFT_AXIS_ENCODER_COUNTS_PER_REV = 1000;					// 250 CPR, 4X (Quad Encoder)	
	public static final double LIFT_AXIS_TRAVEL_DISTANCE_INCHES_PER_REV = 9.375;		// TODO: Need to check this
	
	public static final double LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
									= LEFT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / LEFT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	public static final double RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_COUNT 
									= RIGHT_DRIVE_TRAVEL_DISTANCE_INCHES_PER_REV / RIGHT_DRIVE_ENCODER_COUNTS_PER_REV;
	
	public static final double LIFT_AXIS_TRAVEL_DISTANCE_INCHES_PER_COUNT 
									= LIFT_AXIS_TRAVEL_DISTANCE_INCHES_PER_REV / LIFT_AXIS_ENCODER_COUNTS_PER_REV;
	
	// ======================================
	// define constants for DIO ports on RobioRio
	// ======================================
	public static final int DIO_PORT_TOTE_IN_POSITION_SENSOR = 4;
	
	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int PCM_PORT_CAN_CLAMP_RETRACT = 0;
	public static final int PCM_PORT_CAN_CLAMP_EXTEND = 1;
	public static final int PCM_PORT_FRONT_CLIP_EXTEND = 2;
	public static final int PCM_PORT_FRONT_CLIP_RETRACT = 3;
	
	// ======================================
	// define constants for air cylinder states / positions
	//	(map the physical air cylinder position to logical state)
	// ======================================
	public static final Value CAN_CLAMP_OPEN_POSITION = DoubleSolenoid.Value.kReverse;
	public static final Value CAN_CLAMP_CLOSED_POSITION = DoubleSolenoid.Value.kForward;
	
	public static final Value FRONT_CLIPS_OPEN_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value FRONT_CLIPS_CLOSED_POSITION = DoubleSolenoid.Value.kReverse;
	
	// ======================================
	// define constants for Driver Station Gamepad
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int DRIVER_GAMEPAD_SCALE_SPEED_UP_BTN = LogitechF310.YELLOW_BUTTON_Y;
	public static final int DRIVER_GAMEPAD_SCALE_SPEED_DOWN_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int DRIVER_GAMEPAD_CLIP_OPEN_BTN = LogitechF310.LEFT_BUMPER;
	public static final int DRIVER_GAMEPAD_CLIP_CLOSE_BTN = LogitechF310.RIGHT_BUMPER;
	public static final int DRIVER_GAMEPAD_THROTTLE_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;		
	public static final int DRIVER_GAMEPAD_TURN_AXIS_JOYSTICK = LogitechF310.RIGHT_X_AXIS;		
	
	// ======================================
	// define constants for Operator Station Gamepad
	// ======================================
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	//public static final int OPERATOR_GAMEPAD_LIFT_LIMIT_SWITCH_OVERRIDE_BTN = LogitechF310.GREEN_BUTTON_A;
	//public static final int OPERATOR_GAMEPAD_LIFT_HOLD_POSITION_MODE_BTN = LogitechF310.YELLOW_BUTTON_Y;		//Button Mapping conflicted with Can Clamp/Unclamp Buttons, not sure where to reassign them to
	public static final int OPERATOR_GAMEPAD_CAN_CLAMP_BTN = LogitechF310.YELLOW_BUTTON_Y;
	public static final int OPERATOR_GAMEPAD_CAN_UNCLAMP_BTN = LogitechF310.GREEN_BUTTON_A;
	public static final int OPERATOR_GAMEPAD_LIFT_AXIS_JOYSTICK = LogitechF310.LEFT_Y_AXIS;
	public static final int OPERATOR_GAMEPAD_INFEED_IN_BUTTON = LogitechF310.LEFT_BUMPER;
	public static final int OPERATOR_GAMEPAD_INFEED_OUT_BUTTON = LogitechF310.RIGHT_BUMPER;
	public static final int OPERATOR_GAMEPAD_INFEED_ROTATE_L_TRIGGER = LogitechF310.LEFT_TRIGGER;
	public static final int OPERATOR_GAMEPAD_INFEED_ROTATE_R_TRIGGER = LogitechF310.RIGHT_TRIGGER;
	
	// ======================================
	// define constants for logging
	// ======================================
	public static final String LOG_FILE_PATH = "/media/sda1/logging";
}