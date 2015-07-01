package org.usfirst.frc.team1165.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team1165.robot.subsystems.ReportableSubsystem;
import org.usfirst.frc.team1165.util.Resettable;

/**
 *
 */
public class Resetter extends Command
{
	private Resettable resettable;
	
	public Resetter(Resettable resettable)
	{
		// Use requires() here to declare subsystem dependencies
		this.resettable = resettable;
	}

	// Called just before this Command runs the first time
	protected void initialize()
	{
		resettable.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute()
	{
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished()
	{
		return true;
	}

	// Called once after isFinished returns true
	protected void end()
	{
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted()
	{
	}
}
