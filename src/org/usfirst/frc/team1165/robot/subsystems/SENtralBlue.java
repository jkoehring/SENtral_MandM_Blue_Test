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
	
	public SENtralBlue(I2C.Port port, DigitalInput interrupt)
	{
		blue = new SENtralMandMBlue(port, interrupt);
	}
	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand()
	{
		setDefaultCommand(new Reporter(this));
		SmartDashboard.putData("Reset SENtral Blue", new Resetter(this));
	}
	
	public void report()
	{
		SmartDashboard.putNumber("SENtral Blue Mag Sample Rate", blue.getMagSampleRate());
		SmartDashboard.putNumber("SENtral Blue Accel Sample Rate", blue.getAccelSampleRate());
		SmartDashboard.putNumber("SENtral Blue Gyro Sample Rate", blue.getGyroSampleRate());
		SmartDashboard.putNumber("SENtral Blue HPR Sample Rate", blue.getHprSampleRate());
		
		SmartDashboard.putNumber("SENtral Blue Heading", blue.getHeading());
		SmartDashboard.putNumber("SENtral Blue Pitch", blue.getPitch());
		SmartDashboard.putNumber("SENtral Blue Roll", blue.getRoll());
		
		SmartDashboard.putNumber("Sentral Blue Mag X", blue.getMagX());
		SmartDashboard.putNumber("Sentral Blue Mag Y", blue.getMagY());
		SmartDashboard.putNumber("Sentral Blue Mag Z", blue.getMagZ());
		
		SmartDashboard.putNumber("Sentral Blue Accel X", blue.getAccelX());
		SmartDashboard.putNumber("Sentral Blue Accel Y", blue.getAccelY());
		SmartDashboard.putNumber("Sentral Blue Accel Z", blue.getAccelZ());
		
		SmartDashboard.putNumber("Sentral Blue Gyro X", blue.getGyroX());
		SmartDashboard.putNumber("Sentral Blue Gyro Y", blue.getGyroY());
		SmartDashboard.putNumber("Sentral Blue Gyro Z", blue.getGyroZ());
	}
	
	public void reset()
	{
		blue.initialize();
	}
}
