package org.usfirst.frc.team1165.robot.subsystems;

import org.usfirst.frc.team1165.robot.commands.Reporter;
import org.usfirst.frc.team1165.robot.commands.Resetter;
import org.usfirst.frc.team1165.util.Resettable;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SENtralMandMBlue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A subsystem that gets data from a SENtral M&M Blue.
 */
public class SENtralBlue extends ReportableSubsystem implements Resettable
{
	private SENtralMandMBlue blue;
	
	public SENtralBlue(I2C.Port port)
	{
		this(port, null);
	}
	
	public SENtralBlue(I2C.Port port, DigitalInput interrupt)
	{
		blue = new SENtralMandMBlue(port, interrupt);
	}
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new Reporter(this));
		SmartDashboard.putData("Reset SENtral", new Resetter(this));
	}
	
	public void report()
	{
		SmartDashboard.putNumber("SENtral Mag Sample Rate", blue.getMagSampleRate());
		SmartDashboard.putNumber("SENtral Accel Sample Rate", blue.getAccelSampleRate());
		SmartDashboard.putNumber("SENtral Gyro Sample Rate", blue.getGyroSampleRate());
		SmartDashboard.putNumber("SENtral HPR Sample Rate", blue.getHprSampleRate());
		
		SmartDashboard.putNumber("SENtral Heading", blue.getHeading());
		SmartDashboard.putNumber("SENtral Pitch", blue.getPitch());
		SmartDashboard.putNumber("SENtral Roll", blue.getRoll());
		
		SmartDashboard.putNumber("Sentral Mag X", blue.getMagX());
		SmartDashboard.putNumber("Sentral Mag Y", blue.getMagY());
		SmartDashboard.putNumber("Sentral Mag Z", blue.getMagZ());
		
		SmartDashboard.putNumber("Sentral Accel X", blue.getAccelX());
		SmartDashboard.putNumber("Sentral Accel Y", blue.getAccelY());
		SmartDashboard.putNumber("Sentral Accel Z", blue.getAccelZ());
		
		SmartDashboard.putNumber("Sentral Gyro X", blue.getGyroX());
		SmartDashboard.putNumber("Sentral Gyro Y", blue.getGyroY());
		SmartDashboard.putNumber("Sentral Gyro Z", blue.getGyroZ());
	}
	
	public void reset()
	{
		blue.initialize();
	}
}
