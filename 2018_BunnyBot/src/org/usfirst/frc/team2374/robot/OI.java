/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2374.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	private Joystick driver;
	
	public OI() {
		driver = new Joystick(RobotMap.driverJoy);
	}
	
	public double getDriverLeftY() { return driver.getRawAxis(RobotMap.rsRightAxisY); }
	
	public double getDriverRightY() { return driver.getRawAxis(RobotMap.rsLeftAxisY); }
	
	public double getLeftTrigger() { return driver.getRawAxis(RobotMap.rsLeftTrigger); }

	public double getRightTrigger() { return driver.getRawAxis(RobotMap.rsRightTrigger); }

	public boolean getButtonA() { return driver.getRawButton(RobotMap.rsButtonA); }
	
	public boolean getButtonB() { return driver.getRawButton(RobotMap.rsButtonB); }

	public boolean getButtonX() { return driver.getRawButton(RobotMap.rsButtonX); }

	public boolean getButtonY() { return driver.getRawButton(RobotMap.rsButtonY); }

	public boolean getLeftBumper() { return driver.getRawButton(RobotMap.rsLeftBumper); }

	public boolean getRightBumper() { return driver.getRawButton(RobotMap.rsRightBumper); }
	
	public boolean getButtonBack() { return driver.getRawButton(RobotMap.rsButtonBack); }

	public boolean getButtonBackPressed() { return driver.getRawButtonPressed(RobotMap.rsButtonBack); }
	
	public boolean getButtonStart() { return driver.getRawButton(RobotMap.rsButtonStart); }

	public boolean getButtonStartPressed() { return driver.getRawButtonPressed(RobotMap.rsButtonStart); }
	
	public boolean getButtonM1() { return driver.getRawButton(RobotMap.rsButtonM1); }
	
	public boolean getButtonM1Pressed() { return driver.getRawButtonPressed(RobotMap.rsButtonM1); }
	
	public boolean getButtonM2() { return driver.getRawButton(RobotMap.rsButtonM2); }

	public boolean getButtonM2Pressed() { return driver.getRawButtonPressed(RobotMap.rsButtonM2); }
	
	public double getPOV() { return driver.getPOV(0); }
	
}