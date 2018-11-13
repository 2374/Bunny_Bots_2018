package org.usfirst.frc.team2374.robot.commands;

import org.usfirst.frc.team2374.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DrivetrainTeleop extends Command {
	
	public DrivetrainTeleop() {
		
		requires(Robot.drive);
		
	}
	
	protected void initialize() { }
	
	@Override
	protected void execute() {
		double leftTrigger = Robot.m_oi.getLeftTrigger();
		double rightTrigger = Robot.m_oi.getRightTrigger();
		double leftJoy = Robot.m_oi.getDriverLeftY();
		double rightJoy = Robot.m_oi.getDriverRightY();
		// This is a functionality Alec likes, the left and right triggers can make the robot drive
		// straight forward or straight back
		
		if (rightTrigger != 0) {
			Robot.drive.tankDrive(-rightTrigger + leftJoy, -rightTrigger + rightJoy);
		} else if (leftTrigger != 0) {
			Robot.drive.tankDrive(leftTrigger + leftJoy, leftTrigger + rightJoy);
		} else {
			// This is how normal people drive, the left joystick controls the left side of the
			// drivetrain and the right joystick controls the right side of the drivetrain
			Robot.drive.tankDrive(leftJoy, rightJoy);
		}
	}
	
	@Override
	protected boolean isFinished() { return false; }
	
	@Override
	protected void end() { Robot.drive.tankDrive(0, 0); }
	
	@Override
	protected void interrupted() { end(); }
	
}
