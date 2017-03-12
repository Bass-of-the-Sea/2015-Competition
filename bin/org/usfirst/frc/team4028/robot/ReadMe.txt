 /*
 * Technical References: 
 * 		2015 FRC Control System / Getting Started with JAVA
 * 			http://wpilib.screenstepslive.com/s/4485/m/13809
 * 
 * 		RobRio
 * 			https://decibel.ni.com/content/servlet/JiveServlet/download/30419-60-90614/roboRIO%20User%20Manual.pdf
 * 
 * 		Talon SRX Motor Controller
 * 			v1.6 http://www.ctr-e.com/Talon%20SRX%20Software%20Reference%20Manual.pdf
 * 			v1.5 http://content.vexrobotics.com/vexpro/pdf/Talon-SRX-Software-Reference-Manual-20150226.pdf
 * 
 * 		Tote InPosition Sensor
 * 			http://www.61mcu.com/upload/E18-D80NK-Manual-EN.pdf
 * 
 * 		wpiLIB Documentation Location
 * 			C:\Users\Tom\wpilib\java\current\javadoc\index.html
 * 
 * 		Andymark CIM Motors
 * 			http://www.andymark.com/CIM-motor-FIRST-p/am-0255.htm
 * 				12 VDC
 * 				No Load RPM 5,310 +/- 10%
 * 
 *		navX Robotics Navigation Sensor
 *			http://www.pdocs.kauailabs.com/navx-mxp/
 *
 * SSH or telnet access
 * 	Computer: 172.22.1.2
 *  Username: admin
 *  Password: <blank>
 * 
 * RobioRIO Host USB ports (for USB stick)
 *  - format stick as FAT32 not NTFS
 *  - mounts as /u, /U, /media/sda1
 * 
 * Driver's Station Notes:
 * 		USB 0	Driver's Gamepad
 * 		USB 1	Operator's Gamepad
 * 		Dashboard Type => JAVA
 * 
 * Variable naming conventions used:
 * 
 * 	Type			Description								Example
 * 	-------------	-------------------------------------	---------------------
 * 	constants		All caps, underscores between words		CAN_ADDR_PCM
 * 	class scope 	Initial	underscore, "camel cased"		_ourRobot
 * 	function scope	"camel cased"							isCanClampBtnPressed
 * 
 * Todo:
 *  Basics
 * 	xxx	Format Smart Dashboard
 *	Teleop Advanced
 *  xxx	Wire Up & Test Tote InPosition Sensor
 *  	Hold Lift In Position under load w/ joystick in neutral position	
 * 		Change to Arcade Style Drive
 * 		Field Orientated Driving
 * 		Consider non-linear drive speed control in place of scaling
 * 	Auton Advanced
 * 	xxx	Implement reading values from Smart Dashboard
 * 	xxx	Integrate Navigation Sensor
 * 		Implement PID Control
 * 			Closed Loop Velocity
 * 			Clsoed Loop Position
 * 	xxx	Log to Text file on USB Stick
 * 		Moving
 * 			Straight Line (w/ Nav sensor)
 * 			Stop at commanded position
 * 			Fixed Distance Move
 * 			Turning
 * 				2 wheel L&R Arc
 * 				1 Wheel L or R Arc
 * 				Spin
 * 	xxx	Concurrent Actions
 * 		Text File Config of Auton Steps
 * 	New Stuff
 * 		Motion Profiling
 * 		3 Tote Auton
 * 		Smart Dashboard
 * 	xxx		Read Config Values
 * 	xxx		Write Current Values
 * 		Investigate Command based programming
 * 	Aux Input Devices
 */