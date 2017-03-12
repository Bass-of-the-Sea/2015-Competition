package org.usfirst.frc.team4028.robot;

import java.util.Date;

import org.usfirst.frc.team4028.robot.Constants.RobotMap;
import org.usfirst.frc.team4028.robot.Constants.RobotMap.*;
import org.usfirst.frc.team4028.robot.Robot.*;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
* This class contains the dynamic data while the robot is running in Auton or Teleop
* 	( and follows the DTO [Data Transfer Object] pattern )
* 
* It is used for:
* 	- collection of working data values used by Telop & Auton to control the robot
* 	- data logging to a text file (if enabled)
* 	- sending data to the ShartDashboard
* 
* It contains three (3) inner classes
* 	InputData		all real time data read from the robot (roboRio) 
* 	WorkingData		values calculated from the input data
* 	OutputData		values determined by code that will be pushed back to the robioRio to control motors & solenoids etc.
* 
* Date			Rev		Author						Comments
* -----------	------	-------------------------	---------------------------------- 
* 22.Aug.2015	0.2		Sebastian Rodriguez			Added new fields for Nav sensor
* 02.Aug.2015	0.1		Tom Bruns					Initial Version
*
*/
public class RobotData 
{
	// class constructor
	public RobotData()
	{
		this.InputDataValues = new InputData();
		this.WorkingDataValues = new WorkingData();
		this.OutputDataValues = new OutputData();
	}
	
	// properties
	public InputData InputDataValues;
	public WorkingData WorkingDataValues;
	public OutputData OutputDataValues;
	
	// build a TSV (Tab Separated Value) string for the header
	public String BuildTSVHeader()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVHeader() + "\t");
		sb.append(WorkingDataValues.BuildTSVHeader() + "\t");	
		sb.append(OutputDataValues.BuildTSVHeader() + "\n");
		
		return sb.toString();
	}

	// build a TSV for the data values
	public String BuildTSVData()
	{
		StringBuilder sb = new StringBuilder();
		
		sb.append(InputDataValues.BuildTSVData() + "\t");
		sb.append(WorkingDataValues.BuildTSVData() + "\t");	
		sb.append(OutputDataValues.BuildTSVData() + "\n");
		
		return sb.toString();
	}
	
	// internal class representing all of the Input data (sensors, driver station) used to control the robot
	public class InputData
	{
		public long FPGATimeMicroSecs;
		
		public boolean IsCanClampBtnPressed;					// PCM will latch output value (so this acts like a single shot)
		public boolean IsCanUnclampBtnPressed;					// PCM will latch output value (so this acts like a single shot)
		public boolean IsLiftLimitSwitchOverrideBtnPressed;		// btn must be held
		public boolean IsLiftHoldPositionModeBtnPressed;		// logic will latch output value (so this acts like a toggle)
		
		public boolean IsClipOpenBtnPressed;					// PCM will latch output value (so this acts like a single shot)
		public boolean IsClipClosedBtnPressed;					// PCM will latch output value (so this acts like a single shot)
		public boolean IsScaleDriveSpeedUpBtnPressed;			// logic will latch output value (so this acts like a single shot)
		public boolean IsScaleDriveSpeedDownBtnPressed;			// logic will latch output value (so this acts like a single shot)
		
    	
		public double LiftMtrRawVelocityCmd;
		public double LeftInfeedMtrRawVelocityCmd;
		public double RightInfeedMtrRawVelocityCmd;
		public double ArcadeDriveThrottleRawCmd;
		public double ArcadeDriveTurnRawCmd;
    	    	
		public boolean IsLiftAtTopEOT;
		public boolean IsLiftAtBottomEOT;
		
		public double LiftEncoderCurrentCount;
		public double LeftDriveEncoderCurrentCount;	
		public double RightDriveEncoderCurrentCount;
		
		public boolean IsToteInPosition;
		
		AutonMode AutonModeRequested;
		
		public boolean NavxIsConnected;
		public boolean NavxIsCalibrating;
		public float NavxYaw;
		public float NavxPitch;
		public float NavxRoll;
		public float NavxCompassHeading;
		public float NavxFusedHeading;
		public double NavxTotalYaw;
		public double NavxYawRateDPS;
		public float NavxAccelX;
		public float NavxAccelY;
		public boolean NavxIsMoving;
		public boolean NavxIsRotating;
		
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("InputData:FPGATimeMicroSecs" + "\t");
			sb.append("InputData:IsCanClampBtnPressed" + "\t");
			sb.append("InputData:IsCanUnclampBtnPressed" + "\t");
			sb.append("InputData:IsLiftLimitSwitchOverrideBtnPressed" + "\t");
			sb.append("InputData:IsLiftHoldPositionModeBtnPressed" + "\t");
			sb.append("InputData:IsClipOpenBtnPressed" + "\t");
			sb.append("InputData:IsClipClosedBtnPressed" + "\t");
			sb.append("InputData:IsScaleDriveSpeedUpBtnPressed" + "\t");
			sb.append("InputData:IsScaleDriveSpeedDownBtnPressed" + "\t");
			sb.append("InputData:IsInfeedInBtnPressed" + "\t");
			sb.append("InputData:IsInfeedOutBtnPressed" + "\t");
			sb.append("InputData:LiftMtrRawVelocityCmd" + "\t");
			sb.append("InputData:LeftInfeedMtrRawVelocityCmd" + "\t");
			sb.append("InputData:RightInfeedMtrRawVelocityCmd" + "\t");
			sb.append("InputData:ArcadeDriveThrottleRawCmd" + "\t");
			sb.append("InputData:ArcadeDriveTurnRawCmd" + "\t");
			sb.append("InputData:IsLiftAtTopEOT" + "\t");
			sb.append("InputData:IsLiftAtBottomEOT" + "\t");
			sb.append("InputData:LiftEncoderCurrentCount" + "\t");
			sb.append("InputData:LeftDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:RightDriveEncoderCurrentCount" + "\t");
			sb.append("InputData:IsToteInPosition"+ "\t");
			sb.append("InputData:AutonModeRequested" + "\t");
			
			sb.append("InputData:NavxIsConnected" + "\t");
			sb.append("InputData:NavxIsCalibrating" + "\t");
			sb.append("InputData:NavxYaw" + "\t");
			sb.append("InputData:NavxPitch" + "\t");
			sb.append("InputData:NavxRoll" + "\t");
			sb.append("InputData:NavxCompassHeading" + "\t");
			sb.append("InputData:NavxFusedHeading" + "\t");
			sb.append("InputData:NavxTotalYaw" + "\t");
			sb.append("InputData:NavxYawRateDPS" + "\t");
			sb.append("InputData:NavxAccelX" + "\t");
			sb.append("InputData:NavxAccelY" + "\t");
			sb.append("InputData:NavxIsMoving" + "\t");
			sb.append("InputData:NavxIsRotating");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(FPGATimeMicroSecs + "\t");
			sb.append(IsCanClampBtnPressed + "\t");
			sb.append(IsCanUnclampBtnPressed + "\t");
			sb.append(IsLiftLimitSwitchOverrideBtnPressed + "\t");
			sb.append(IsLiftHoldPositionModeBtnPressed + "\t");
			sb.append(IsClipOpenBtnPressed + "\t");
			sb.append(IsClipClosedBtnPressed + "\t");
			sb.append(IsScaleDriveSpeedUpBtnPressed + "\t");
			sb.append(IsScaleDriveSpeedDownBtnPressed + "\t");
			sb.append(IsInfeedInBtnPressed + "\t");
			sb.append(IsInfeedOutBtnPressed + "\t");
			sb.append(LiftMtrRawVelocityCmd + "\t");
			sb.append(LeftInfeedMtrRawVelocityCmd + "\t");
			sb.append(RightInfeedMtrRawVelocityCmd + "\t");
			sb.append(ArcadeDriveThrottleRawCmd + "\t");
			sb.append(ArcadeDriveTurnRawCmd + "\t");
			sb.append(IsLiftAtTopEOT + "\t");
			sb.append(IsLiftAtBottomEOT + "\t");
			sb.append(LiftEncoderCurrentCount + "\t");
			sb.append(LeftDriveEncoderCurrentCount + "\t");
			sb.append(RightDriveEncoderCurrentCount + "\t");

			sb.append(IsToteInPosition + "\t");
			sb.append(AutonModeRequested + "\t");
					
			sb.append(NavxIsConnected + "\t");
			sb.append(NavxIsCalibrating + "\t");
			sb.append(NavxYaw + "\t");
			sb.append(NavxPitch + "\t");
			sb.append(NavxRoll + "\t");
			sb.append(NavxCompassHeading + "\t");
			sb.append(NavxFusedHeading + "\t");
			sb.append(NavxTotalYaw + "\t");
			sb.append(NavxYawRateDPS + "\t");
			sb.append(NavxAccelX + "\t");
			sb.append(NavxAccelY + "\t");
			sb.append(NavxIsMoving + "\t");
			sb.append(NavxIsRotating);

			return sb.toString();
		}
	}
	
	// internal class representing all of the working data
	public class WorkingData
	{
		public boolean IsLoggingEnabled;
		public String LogFilePathName;
		public Date LoggingStartedDT;
		
		public String CurrentAutonState;
		
		public Date LastScanDT;
		
		public boolean IsDriveSpeedScalingButtonPressedLastScan;
		public double DriveSpeedScalingFactor;			// min = 0.0, max = 1.0, 1.0 = 100%, 
		public boolean IsLiftHoldPositionButtonPressedLastScan;
		public boolean IsLiftHoldPositionModeEnabled;
		public boolean IsLiftHoldPositionModeActive;
		
		public double LiftEncoderInitialCount;
		public double LiftEncoderLastCount;
		public double LiftEncoderLastDeltaCount;
		public double LiftEncoderTotalDeltaCount;
		
		public double LiftAxisMotorCurrentRPM;
		public double LiftAxisEncoderCurrentCPS;
		public double LiftAxisGearBoxCurrentRPM;
		public double LiftAxisCurrentSpeedIPS;
		
		public double LiftMtrLastScanRawVelocityCmd;
		public double LiftEncoderHoldTargetCount;
		
		public double LeftDriveEncoderInitialCount;
		public double LeftDriveEncoderLastCount;
		public double LeftDriveEncoderLastDeltaCount;
		public double LeftDriveEncoderTotalDeltaCount;
		
		public double LeftDriveMotorCurrentRPM;
    	public double LeftDriveEncoderCurrentCPS;
    	public double LeftDriveGearBoxCurrentRPM;
    	public double LeftDriveWheelsCurrentSpeedIPS;
		
		public double RightDriveEncoderInitialCount;
		public double RightDriveEncoderLastCount;
		public double RightDriveEncoderLastDeltaCount;
		public double RightDriveEncoderTotalDeltaCount;
		
		public double RightDriveMotorCurrentRPM;
    	public double RightDriveEncoderCurrentCPS;
    	public double RightDriveGearBoxCurrentRPM;
    	public double RightDriveWheelsCurrentSpeedIPS;
		
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("WorkingData:CurrentAutonState" + "\t");
			sb.append("WorkingData:IsDriveSpeedScalingButtonPressedLastScan" + "\t");
			sb.append("WorkingData:DriveSpeedScalingFactor" + "\t");
			sb.append("WorkingData:IsLiftHoldPositionButtonPressedLastScan" + "\t");
			sb.append("WorkingData:IsLiftHoldPositionModeEnabled" + "\t");
			sb.append("WorkingData:IsLiftHoldPositionModeActive" + "\t");
			
			sb.append("WorkingData:LiftEncoderInitialCount" + "\t");
			sb.append("WorkingData:LiftEncoderLastCount" + "\t");
			sb.append("WorkingData:LiftEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:LiftEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:LiftMotorCurrentRPM" + "\t");
			sb.append("WorkingData:LiftAxisEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:LiftAxisGearBoxCurrentRPM" + "\t");
			sb.append("WorkingData:LiftAxisCurrentSpeedIPS" + "\t");
			
			sb.append("WorkingData:LiftMtrLastScanRawVelocityCmd" + "\t");
			sb.append("WorkingData:LiftEncoderHoldTargetCount" + "\t");
			
			sb.append("WorkingData:LeftDriveEncoderInitialCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderLastCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:LeftDriveEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:LeftDriveMotorCurrentRPM" + "\t");
			sb.append("WorkingData:LeftDriveEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:LeftDriveGearBoxCurrentRPM" + "\t");
			sb.append("WorkingData:LeftDriveWheelsCurrentSpeedIPS" + "\t");
			
			sb.append("WorkingData:RightDriveEncoderInitialCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderLastCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderLastDeltaCount" + "\t");
			sb.append("WorkingData:RightDriveEncoderTotalDeltaCount" + "\t");
			
			sb.append("WorkingData:RightDriveMotorCurrentRPM" + "\t");
			sb.append("WorkingData:RightDriveEncoderCurrentCPS" + "\t");
			sb.append("WorkingData:RightDriveGearBoxCurrentRPM" + "\t");
			sb.append("WorkingData:RightDriveWheelsCurrentSpeedIPS");
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(CurrentAutonState + "\t");
			sb.append(IsDriveSpeedScalingButtonPressedLastScan + "\t");
			sb.append(DriveSpeedScalingFactor + "\t");
			sb.append(IsLiftHoldPositionButtonPressedLastScan + "\t");
			sb.append(IsLiftHoldPositionModeEnabled + "\t");
			sb.append(IsLiftHoldPositionModeActive + "\t");
			
			sb.append(LiftEncoderInitialCount + "\t");
			sb.append(LiftEncoderLastCount + "\t");
			sb.append(LiftEncoderLastDeltaCount + "\t");
			sb.append(LiftEncoderTotalDeltaCount + "\t");
			
			sb.append(LiftAxisMotorCurrentRPM + "\t");
			sb.append(LiftAxisEncoderCurrentCPS + "\t");
			sb.append(LiftAxisGearBoxCurrentRPM + "\t");
			sb.append(LiftAxisCurrentSpeedIPS + "\t");
			
			sb.append(LiftMtrLastScanRawVelocityCmd + "\t");
			sb.append(LiftEncoderHoldTargetCount  + "\t");
			
			sb.append(LeftDriveEncoderInitialCount + "\t");
			sb.append(LeftDriveEncoderLastCount + "\t");
			sb.append(LeftDriveEncoderLastDeltaCount + "\t");
			sb.append(LeftDriveEncoderTotalDeltaCount + "\t");
			
			sb.append(LeftDriveMotorCurrentRPM + "\t");
			sb.append(LeftDriveEncoderCurrentCPS + "\t");
			sb.append(LeftDriveGearBoxCurrentRPM + "\t");
			sb.append(LeftDriveWheelsCurrentSpeedIPS + "\t");
			
			sb.append(RightDriveEncoderInitialCount + "\t");
			sb.append(RightDriveEncoderLastCount + "\t");
			sb.append(RightDriveEncoderLastDeltaCount + "\t");
			sb.append(RightDriveEncoderTotalDeltaCount + "\t");
			
			sb.append(RightDriveMotorCurrentRPM + "\t");
			sb.append(RightDriveEncoderCurrentCPS + "\t");
			sb.append(RightDriveGearBoxCurrentRPM + "\t");
			sb.append(RightDriveWheelsCurrentSpeedIPS);
					
			return sb.toString();
		}
	}
	
	// internal class representing all of the Motor Output Data used to control the robot
	public class OutputData
	{
		public double ArcadeDriveThrottleAdjCmd;
		public double ArcadeDriveTurnAdjCmd;
		
		public double LiftMtrAdjVelocityCmd;
		
		public double LeftInfeedMtrAdjVelocityCmd;
		public double RightInfeedMtrAdjVelocityCmd;
		
		public Value CanClampPosition;
		public Value FrontClipPosition;
		
		public String DriversStationMsg;
		
		// build a TSV for the header
		public String BuildTSVHeader()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append("OutputData:ArcadeDriveAdjThrottleCmd" + "\t");
			sb.append("OutputData:ArcadeDriveAdjTurnCmd" + "\t");
			sb.append("OutputData:LiftMtrAdjVelocityCmd" + "\t");
			sb.append("OutputData:LeftInfeedMtrAdjVelocityCmd" + "\t");
			sb.append("OutputData:RightInfeedMtrAdjVelocityCmd" + "\t");
			sb.append("OutputData:CanClampPosition" + "\t");
			sb.append("OutputData:FrontClipPosition" + "\t");
			sb.append("OutputData:DriversStationMsg");
			
					
			return sb.toString();
		}
		
		// build a TSV for the data
		public String BuildTSVData()
		{
			StringBuilder sb = new StringBuilder();
			
			sb.append(ArcadeDriveThrottleAdjCmd + "\t");
			sb.append(ArcadeDriveTurnAdjCmd + "\t");
			sb.append(LiftMtrAdjVelocityCmd + "\t");
			sb.append(LeftInfeedMtrAdjVelocityCmd + "\t");
			sb.append(RightInfeedMtrAdjVelocityCmd + "\t");
			
			String canClampPositionDesc = "";
			if (CanClampPosition == RobotMap.CAN_CLAMP_OPEN_POSITION)
			{
				canClampPositionDesc = "CAN_CLAMP_OPEN";
			}
			else if (CanClampPosition == RobotMap.CAN_CLAMP_CLOSED_POSITION)
			{
				canClampPositionDesc = "CAN_CLAMP_CLOSED";
			}
			else
			{
				canClampPositionDesc = "UNKNOWN";
			}
			
			String frontClipsPositionDesc = "";
			if (FrontClipPosition == RobotMap.FRONT_CLIPS_OPEN_POSITION)
			{
				frontClipsPositionDesc = "FRONT_CLIPS_OPEN";
			}
			else if (FrontClipPosition == RobotMap.FRONT_CLIPS_CLOSED_POSITION)
			{
				frontClipsPositionDesc = "FRONT_CLIPS_CLOSED";
			}
			else
			{
				frontClipsPositionDesc = "UNKNOWN";
			}
			
			sb.append(canClampPositionDesc + "\t");
			sb.append(frontClipsPositionDesc + "\t");
			sb.append(DriversStationMsg);
					
			return sb.toString();
		}
	}
}
