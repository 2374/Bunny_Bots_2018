/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2374.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	
	
	// Driver Joystick
	public static int driverJoy = 0;
	
	// Joystick Buttons
	public static int rsLeftAxisY = 1;
	public static int rsRightAxisY = 5;
	public static int rsLeftTrigger = 2;
	public static int rsRightTrigger = 3;
	public static int rsButtonA = 1;
	public static int rsButtonB = 2;
	public static int rsButtonX = 3;
	public static int rsButtonY = 4;
	public static int rsLeftBumper = 5;
	public static int rsRightBumper = 6;
	public static int rsButtonBack = 7;
	public static int rsButtonStart = 8;
	public static int rsButtonM1 = 9;
	public static int rsButtonM2 = 10;
	
	// Drivetrain CIMS
	public static final int TALON_DRIVE_FRONT_LEFT = 0;
	public static final int TALON_DRIVE_MIDDLE_LEFT = 1;
	public static final int TALON_DRIVE_BACK_LEFT = 2;
	public static final int TALON_DRIVE_FRONT_RIGHT = 3;
	public static final int TALON_DRIVE_MIDDLE_RIGHT = 4;
	public static final int TALON_DRIVE_BACK_RIGHT = 5;
}