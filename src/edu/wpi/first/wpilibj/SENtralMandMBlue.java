package edu.wpi.first.wpilibj;

import java.io.PrintWriter;

import org.usfirst.frc.team1165.util.SampleRate;

import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Implements an interface to the PNi SENtral M&M Blue motion and measurement module.
 * 
 * @author jkoehring
 *
 */
public class SENtralMandMBlue extends SensorBase
{
	/**
	 * The address on the I2C bus.
	 */
	public static final int ADDRESS = 0b0101000;
	
	/* Configuration File Upload From EEPROM Registers */
	
	/**
	 * Status Register (read):
	 *   [0] EEPROM: 1 = EEPROM detected
	 *   [1] EEUploadDone:  = EEPROM upload completed
	 *   [2] EEUploadError: 1 = Calculated CRC of EEPROM is incorrect
	 *   [3] Idle: 1 = Device in un-programmed or Initialized state
	 *   [4] NoEEPROM: 1 = No EEPROM detected
	 */
	public static final int SENTRAL_STATUS_REGISTER = 0x37;
	public static final int SENTRAL_STATUS_EEPROM_DETECTED				= 0b00000001;
	public static final int SENTRAL_STATUS_EEPROM_UPLOAD_COMPLETED		= 0b00000010;
	public static final int SENTRAL_STATUS_EEPROM_EEPROM_UPLOAD_ERROR	= 0b00000100;
	public static final int SENTRAL_STATUS_IDLE							= 0b00001000;
	public static final int SENTRAL_STATUS_NO_EEPRON					= 0b00010000;
	
	/**
	 * ResetReq Register (write):
	 *   [0] ResetRequest: 1 = Emulate a hard power down/power up
	 */
	public static final int RESET_REQ_REGISTER = 0x9B;
	
	/**
	 * MagRate Register (read/write):
	 *   Requested magnetometer output data rate
	 */
	public static final int MAG_RATE_REGISTER = 0x55;
	
	/* Registers for Initial Set-up */
	
	/**
	 * AccelRate Register (read/write):
	 *   Requested accelerometer output data rate divided by 10
	 */
	public static final int ACCEL_RATE_REGISTER = 0x56;
	
	/**
	 * GyroRate Register (read/write):
	 *   Requested gyroscope output data rate divided by 10
	 */
	public static final int GYRO_RATE_REGISTER = 0x57;
	
	/**
	 * QRateDivisor Register (read/write):
	 *   Along with GyroRate, establishes output data rate for quaternion date
	 */
	public static final int Q_RATE_DIVISOR_REGISTER = 0x32;
	
	/** AlgorithmControl Register (write):
	 *   [0] 1 = Enable Standby State
	 *       0 = Disable Standby State (return to Normal Operation)
	 *   [1] RawDataEnable
	 *       1 = Raw data provided in MX, MY, MZ, AZ, AY, AZ, GX, GY & GZ
	 *       0 = Scaled sensor data
	 *   [2] HPRoutput
	 *       1 = Heading pitch and roll output in QX, QY, QZ; QW = 0.0
	 *       0 = Quaternion outputs
	 */
	public static final int ALGORITHM_CONTROL_REGISTER = 0x54;
	public static final int ALGORITHM_CONTROL_ENABLE_STANDBY	= 0b00000001;
	public static final int ALGORITHM_CONTROL_RAW_DATA_ENABLE	= 0b00000010;
	public static final int ALGORITHM_CONTROL_HPR_OUTPUT		= 0b00000100;
	
	/** AlgorithmStatus Register (read):
	 *   [0] 1 = SENtral in Standby State
	 *       0 = SENtral not in Standby State
	 */
	public static final int ALGORITHM_STATUS_REGISTER = 0x38;
	
	/**
	 * EnableEvents Register:
	 *   '1' indicates an interrupt to the host will be generated for the event
	 *   [0] CPUReset; non-maskable
	 *   [1] Error
	 *   [2] QuaternionResult
	 *   [3] MagResult
	 *   [4] AccelResult
	 *   [5] GyroResult
	 */
	public static final int ENABLE_EVENTS_REGISTER = 0x33;
	public static final int ENABLE_EVENTS_CPU_RESET			= 0b00000001;
	public static final int ENABLE_EVENTS_ERROR				= 0b00000010;
	public static final int ENABLE_EVENTS_QUATERNION_RESULT	= 0b00000100;
	public static final int ENABLE_EVENTS_MAG_RESULT		= 0b00001000;
	public static final int ENABLE_EVENTS_ACCEL_RESULT		= 0b00010000;
	public static final int ENABLE_EVENTS_GYRO_RESULT		= 0b00100000;
	public static final int ENABLE_EVENTS_ALL				= 0b00111111;
	
	/* Normal Operation Registers */
	
	/**
	 * HostControl Register (write):
	 *   [0] 1 = Run enable
	 *       0 = Enable initialized state; Standby State generally is preferred
	 *           since enabling Initialized State resets the SENtral algorithm,
	 *           including calibration data
	 */
	public static final int HOST_CONTROL_REGISTER = 0x34;
	public static final int HOST_CONTROL_RUN_ENABLE = 0b00000001;
	
	/**
	 * EventStatus Register (read):
	 *   '1' indicates a new event has been generated
	 *   [0] CPUReset
	 *   [1] Error
	 *   [2] QuaternionResult
	 *   [3] MagResult
	 *   [4] AccelResult
	 *   [5] GyroResult
	 */
	public static final int EVENT_STATUS_REGISTER = 0x35;
	public static final int EVENT_STATUS_CPU_RESET			= 0b00000001;
	public static final int EVENT_STATUS_ERROR				= 0b00000010;
	public static final int EVENT_STATUS_QUATERNION_RESULT	= 0b00000100;
	public static final int EVENT_STATUS_MAG_RESULT			= 0b00001000;
	public static final int EVENT_STATUS_ACCEL_RESULT		= 0b00010000;
	public static final int EVENT_STATUS_GYRO_RESULT		= 0b00100000;
	
	/* Results Registers */
	
	/**
	 * QX Register (read):
	 *   Value: Normalized Quaternion - X, or Heading
	 *   Range: 0.0 - 1.0, or +/-Pi
	 *   Format: Float32, little-endian
	 */
	public static final int QX_REGISTER = 0x00;
	
	/**
	 * QY Register (read):
	 *   Value: Normalized Quaternion - Y, or Pitch
	 *   Range: 0.0 - 1.0, or +/-(Pi/2)
	 *   Format: Float32, little-endian
	 */
	public static final int QY_REGISTER = 0x04;
	
	/**
	 * QZ Register (read):
	 *   Value: Normalized Quaternion - Z, or Roll
	 *   Range: 0.0 - 1.0, or +/-Pi
	 *   Format: Float32, little-endian
	 */
	public static final int QZ_REGISTER = 0x08;
	
	/**
	 * QW Register (read):
	 *   Value: Normalized Quaternion - W, or 0.0
	 *   Range: 0.0 - 1.0
	 *   Format: Float32, little-endian
	 */
	public static final int QW_REGISTER = 0x0C;
	
	/**
	 * QTime Register (read):
	 *   Value: Quaternion Data Timestamp
	 *   Range: 0 - 2048 msec
	 *   Format: UInt16, little-endian
	 */
	public static final int QTIME_REGISTER = 0x10;
	
	/**
	 * MX Register (read):
	 *   Value: Magnetic Field - X Axis, or Raw Mag Data
	 *   Range: +/-1000 uT when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int MX_REGISTER = 0x12;
	
	/**
	 * MY Register (read):
	 *   Value: Magnetic Field - Y Axis, or Raw Mag Data
	 *   Range: +/-1000 uT when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int MY_REGISTER = 0x14;
	
	/**
	 * MZ Register (read):
	 *   Value: Magnetic Field - Z Axis, or Raw Mag Data
	 *   Range: +/-1000 uT when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int MZ_REGISTER = 0x16;
	
	/**
	 * MTime Register (read):
	 *   Value: Magnetometer Interrupt Timestamp
	 *   Range: 0 - 2048 msec
	 *   Format: UInt16, little-endian
	 */
	public static final int MTIME_REGISTER = 0x18;
	
	/**
	 * AX Register (read):
	 *   Value: Linear Acceleration - X Axis, or Raw Accel Data
	 *   Range: +/-16 g when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int AX_REGISTER = 0x1A;
	
	/**
	 * AY Register (read):
	 *   Value: Linear Acceleration - Y Axis, or Raw Accel Data
	 *   Range: +/-16 g when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int AY_REGISTER = 0x1C;
	
	/**
	 * AZ Register (read):
	 *   Value: Linear Acceleration - Z Axis, or Raw Accel Data
	 *   Range: +/-16 g when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int AZ_REGISTER = 0x1E;
	
	/**
	 * ATime Register (read):
	 *   Value: Accelerometer Interrupt Timestamp
	 *   Range: 0 - 2048 msec
	 *   Format: UInt16, little-endian
	 */
	public static final int ATIME_REGISTER = 0x20;
	
	/**
	 * GX Register (read):
	 *   Value: Rotational Velocity - X Axis, or Raw Gyro Data
	 *   Range: +/-5000 degrees/sec when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int GX_REGISTER = 0x22;
	
	/**
	 * GY Register (read):
	 *   Value: Rotational Velocity - Y Axis, or Raw Gyro Data
	 *   Range: +/-5000 degrees/sec when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int GY_REGISTER = 0x24;
	
	/**
	 * GZ Register (read):
	 *   Value: Rotational Velocity - Z Axis, or Raw Gyro Data
	 *   Range: +/-5000 degrees/sec when scaled
	 *   Format: Int16, little-endian
	 */
	public static final int GZ_REGISTER = 0x26;
	
	/**
	 * GTime Register (read):
	 *   Value: Gyroscope Interrupt Timestamp
	 *   Range: 0 - 2048 msec
	 *   Format: UInt16, little-endian
	 */
	public static final int GTIME_REGISTER = 0x28;
	
	/* Pass-Through Registers */
	
	/**
	 * PassThroughControl Register (write):
	 *   [0] 1 = Enable Pass-Through State; SENtral should be put in Standby State first
	 *       0 = Disable Pass-Through State; SENtral should then be removed from Standby State
	 */
	public static final int PASS_THROUGH_CONTROL_REGISTER = 0xA0;
	
	/**
	 * PassThroughStatus Register (read):
	 *   [0] 1 = SENtral in Pass-Through State
	 *       0 = SENtral not in Pass-Through State
	 */
	public static final int PASS_THROUGH_STATUS_REGISTER = 0x9E;
	
	/* Software-Related Error Indications */

	/**
	 * SensorStatus Register:
	 *   [0] MagNACK: 1 = NACK from magnetometer
	 *   [1] AccelNACK: 1 = NACK from accelerometer
	 *   [2] GryoNACK: 1 = NACK from gyroscope
	 *   [3] MagDeviceIDErr: 1 = Unexpected DeviceID from magnetometer
	 *   [4] AccelDeviceIDErr: 1 = Unexpected DeviceID from accelerometer
	 *   [5] GyroDeviceIDErr: 1 = Unexpected DeviceID from gyroscope
	 */
	public static final int SENSOR_STATUS_REGISTER = 0x36;
	
	/**
	 * Error Register:
	 *   0x00 - No error
	 *   0x80 - Invalid sample rate selected
	 *   0x30 - Mathematical error
	 *   0x21 - Magnetometer initialization failed
	 *   0x22 - Accelerometer initialization failed
	 *   0x24 - Gyroscope initialization failed
	 *   0x11 - Magnetometer rate failure
	 *   0x12 - Accelerometer rate failure
	 *   0x14 - Gyroscope rate failure
	 */
	public static final int ERROR_REGISTER = 0x50;
	
	/**
	 * RAMVersion Register:
	 *   Range:
	 *     0x0C04 / 3076 - 1.0
	 *     0x0CD5 / 3285 - 1.1
	 *     0x0E02 / 3639 - 1.2
	 *   Format: UInt16, little-endian
	 */
	public static final int RAM_VERSION_REGISTER = 0x72;
	
	/**
	 * Various sample rates:
	 */
	public static final int MAG_RATE		= 0x64;	// 100 Hz
	public static final int ACCEL_RATE		= 0x0A; // * 10 = 100 Hz
	public static final int GYRO_RATE		= 0x0F; // * 10 = 150 Hz
	public static final int Q_RATE_DIVISOR	= 0x01; // GYRO_RATE / 1 = 100 Hz
	
	/**
	 * Scale factors:
	 */
	public static final double MAG_SCALE_FACTOR		= 32768.0 / 1000.0;
	public static final double ACCEL_SCALE_FACTOR	= 32768.0 / 16.0;
	public static final double GYRO_SCALE_FACTOR	= 32768.0 / 5000.0;
	
	/**
	 * The I2C object used to communicate.
	 */
	private FixedI2C i2c;
	
	/**
	 * HPR data and access control
	 * Units: degrees
	 */
	private double heading;
	private double pitch;
	private double roll;
	private Object hprLock = new Object();
	
	/**
	 * Gyroscope data and access control.
	 * Units: degrees / sec
	 */
	private double gyroX;
	private double gyroY;
	private double gyroZ;
	private Object gyroLock = new Object();
	
	/**
	 * Magnetometer data and access control.
	 * Units: microtesla
	 */
	private double magX;
	private double magY;
	private double magZ;
	private Object magLock = new Object();
	
	/**
	 * Accelerometer data and access control.
	 * Unit: g
	 */
	private double accelX;
	private double accelY;
	private double accelZ;
	private Object accelLock = new Object();
	
	/**
	 * The following keep track of sample rates:
	 */
	private SampleRate hprSampleRate = new SampleRate();
	private SampleRate magSampleRate = new SampleRate();
	private SampleRate accelSampleRate = new SampleRate();
	private SampleRate gyroSampleRate = new SampleRate();
	
	/**
	 * The following is used for receiving interrupts fomr the device.
	 */
	private DigitalSource interrupt;
	private boolean isDataProcessingEnabled;

	// This file on the roboRIO file system is used to log important M&M Blue events and errors:
	private final static String logFile = "/home/lvuser/data/SENtral_MandM_Blue.txt";
	private PrintWriter logPw;
	
	/**
	 * Constructor. Creates and initializes the device, including enabling the sensors and SENtral algorithm.
	 * The SENtral is polled for data.
	 */
	public SENtralMandMBlue(Port port)
	{
		this(port, null);
	}
	
	/**
	 * Constructor. Creates and initializes the device, including enabling the sensors and SENtral algorithm.
	 * If interrupt is not null, interrupts are used to get data from the SENtral.
	 */
	public SENtralMandMBlue(Port port, DigitalSource interrupt)
	{
		// Create I2C device:
		i2c = new FixedI2C(port, ADDRESS);
		
		// Register an interrupt handler:
		this.interrupt = interrupt;
		isDataProcessingEnabled = false;
		if (interrupt != null)
		{
			interrupt.requestInterrupts(new InterruptHandlerFunction<Object>()
			{
				@Override
				public void interruptFired(int interruptAssertedMask, Object param)
				{
					handleInterrupt();
				}
			});
			interrupt.setUpSourceEdge(true, false);
		}
		
		// Open log file:
		try
		{
			logPw = new PrintWriter(logFile);
		}
		catch (Exception ex)
		{
			// ignore
		}

		// Initialize the device:
		initialize();
	}
	
	/**
	 * Disables processing data from the SENtral.
	 */
	private void disableDataProcessing()
	{
		if (isDataProcessingEnabled)
		{
			if (interrupt != null)
			{
				interrupt.disableInterrupts();
			}
			isDataProcessingEnabled = false;
		}
	}
	
	/**
	 * Enables the processing of data from the SENtral.
	 */
	private void enableDataProcessing()
	{
		if (!isDataProcessingEnabled)
		{
			isDataProcessingEnabled = true;
			if (interrupt != null)
			{
				interrupt.enableInterrupts();
			}
			else
			{
				new Thread()
				{
					@Override
					public void run()
					{
						while (isDataProcessingEnabled)
						{
							poll();
							try
							{
								Thread.sleep(10);
							}
							catch (InterruptedException e)
							{
								// ignore
							}
						}
					}
				}.start();
			}
		}
	}
	
	/**
	 * Returns X axis acceleration with a range of +/-16 g.
	 */
	public double getAccelX()
	{
		synchronized(accelLock)
		{
			return accelX;
		}
	}
	
	/**
	 * returns the accelerometer sample rate.
	 */
	public double getAccelSampleRate()
	{
		return accelSampleRate.getSampleRate();
	}
	
	/**
	 * Returns Y axis acceleration with a range of +/-16 g.
	 */
	public double getAccelY()
	{
		synchronized(accelLock)
		{
			return accelY;
		}
	}
	
	/**
	 * Returns Z axis acceleration with a range of +/-16 g.
	 */
	public double getAccelZ()
	{
		synchronized(accelLock)
		{
			return accelZ;
		}
	}
	
	/**
	 * returns the gyroscope sample rate.
	 */
	public double getGyroSampleRate()
	{
		return gyroSampleRate.getSampleRate();
	}
	
	/**
	 * Returns rotational velocity around X axis with a range of +/-5000 degrees/sec.
	 */
	public double getGyroX()
	{
		synchronized(gyroLock)
		{
			return gyroX;
		}
	}
	
	/**
	 * Returns rotational velocity around Y axis with a range of +/-5000 degrees/sec.
	 */
	public double getGyroY()
	{
		synchronized(gyroLock)
		{
			return gyroY;
		}
	}
	
	/**
	 * Returns rotational velocity around Z axis with a range of +/-5000 degrees/sec.
	 */
	public double getGyroZ()
	{
		synchronized(gyroLock)
		{
			return gyroZ;
		}
	}
	
	/**
	 * Returns heading in radians with a range of +/-Pi.
	 */
	public double getHeading()
	{
		synchronized(hprLock)
		{
			return heading;
		}
	}
	
	/**
	 * returns the heading/pitch/roll sample rate.
	 */
	public double getHprSampleRate()
	{
		return hprSampleRate.getSampleRate();
	}
	
	/**
	 * returns the magnetometer sample rate.
	 */
	public double getMagSampleRate()
	{
		return magSampleRate.getSampleRate();
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public double getMagX()
	{
		synchronized(magLock)
		{
			return magX;
		}
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public double getMagY()
	{
		synchronized(magLock)
		{
			return magY;
		}
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public double getMagZ()
	{
		synchronized(magLock)
		{
			return magZ;
		}
	}
	
	/**
	 * Returns pitch in radians with a range of +/-(Pi/2).
	 */
	public double getPitch()
	{
		synchronized(hprLock)
		{
			return pitch;
		}
	}
	
	/**
	 * Returns roll in radians with a range of +/-Pi.
	 */
	public double getRoll()
	{
		synchronized(hprLock)
		{
			return roll;
		}
	}
	
	/**
	 * Handles processing events from the device.
	 * Also used to process manufactured events when polling.
	 */
	private void handleEvents(int events)
	{
		if ((events & EVENT_STATUS_CPU_RESET) != 0)
		{
			log("Received CPU Reset interrupt event");
		}
		
		if ((events & EVENT_STATUS_ERROR) != 0)
		{
			int sensorStatus = readByte(SENSOR_STATUS_REGISTER);
			int error = readByte(ERROR_REGISTER);
			log(String.format("Received Error interrupt event: Sensor Status = %02X, Error = %02X", sensorStatus, error));
		}
		
		if ((events & EVENT_STATUS_QUATERNION_RESULT) != 0)
		{
			synchronized(hprLock)
			{
				hprSampleRate.addSample();
				heading = (float) Math.toDegrees(readFloat32(QX_REGISTER));
				pitch = (float) Math.toDegrees(readFloat32(QY_REGISTER));
				roll = (float) Math.toDegrees(readFloat32(QZ_REGISTER));
			}
		}
	
		if ((events & EVENT_STATUS_MAG_RESULT) != 0)
		{
			synchronized(magLock)
			{
				magSampleRate.addSample();
				int[] data = new int[3];
				readInt16s(MX_REGISTER, data);
				magX = data[0] / MAG_SCALE_FACTOR;
				magY = data[1] / MAG_SCALE_FACTOR;
				magZ = data[2] / MAG_SCALE_FACTOR;
			}
		}
		
		if ((events & EVENT_STATUS_ACCEL_RESULT) != 0)
		{
			synchronized(accelLock)
			{
				accelSampleRate.addSample();
				int[] data = new int[3];
				readInt16s(AX_REGISTER, data);
				accelX = data[0] / ACCEL_SCALE_FACTOR;
				accelY = data[1] / ACCEL_SCALE_FACTOR;
				accelZ = data[2] / ACCEL_SCALE_FACTOR;
			}
		}

		if ((events & EVENT_STATUS_GYRO_RESULT) != 0)
		{
			synchronized(gyroLock)
			{
				gyroSampleRate.addSample();
				int[] data = new int[3];
				readInt16s(GX_REGISTER, data);
				gyroX = data[0] / GYRO_SCALE_FACTOR;
				gyroY = data[1] / GYRO_SCALE_FACTOR;
				gyroZ = data[2] / GYRO_SCALE_FACTOR;
			}
		}
		
	}
	
	/**
	 * Handles an interrupt from the device.
	 */
	private void handleInterrupt()
	{
		synchronized(interrupt)
		{
			handleEvents(readByte(EVENT_STATUS_REGISTER));
		}
	}
	
	/**
	 * Initializes the device and enables the sensors and SENtral algorithm.
	 */
	public void initialize()
	{
		disableDataProcessing();
		if (!resetDevice())
		{
			return;
		}
		
		if (!setSampleRates())
		{
			return;
		}
		
		if (!setAlgorithmControl())
		{
			return;
		}
		
		if (!setEnabledEvents())
		{
			return;
		}
		
		run();
	}
	
	private void log(String msg)
	{
		if (logPw != null)
		{
			logPw.println(msg);
			logPw.flush();
		}
	}
	
	/**
	 * Polls the SENtral for new data.
	 */
	private void poll()
	{
		handleEvents(readByte(EVENT_STATUS_REGISTER));
	}
	
	/**
	 * Reads enough bytes from the I2C device at the specified address to fill the specified buffer.
	 */
	private void readI2C(int address, byte[] buffer)
	{
		if (i2c.read(address,  buffer.length, buffer))
		{
			log(String.format("Failed to read %d bytes from 0x%02X", buffer.length, address));
		}
	}
	
	/**
	 * Reads a single, unsigned, byte of data from the I2C device at the specified register address.
	 */
	private int readByte(int address)
	{
		byte[] buffer = new byte[1];
		readI2C(address, buffer);
		return buffer[0] & 0xff;
	}
	
	/**
	 * Reads a 32-bit floating point value from the I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	private float readFloat32(int address)
	{
		byte[] buffer = new byte[4];
		readI2C(address, buffer);
		return Float.intBitsToFloat(((buffer[3] & 0xff) << 24) | ((buffer[2] & 0xff) << 16) | ((buffer[1] & 0xff) << 8) | (buffer[0] & 0xff));
	}
	
	/**
	 * Reads a signed 16-bit integer value from the I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	@SuppressWarnings("unused")
	private int readInt16(int address)
	{
		byte[] buffer = new byte[2];
		readI2C(address, buffer);
		return (buffer[1] << 8) | (buffer[0] & 0xff);
	}
	
	/**
	 * Reads an array of signed 16-bit integer values from the I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	private void readInt16s(int address, int[] data)
	{
		byte[] buffer = new byte[data.length * 2];
		readI2C(address, buffer);
		for (int i = 0; i < data.length; i++)
		{
			data[i] = (buffer[i * 2 + 1] << 8) | (buffer[i * 2] & 0xff);
		}
	}
	
	/**
	 * Reads an un-signed 16-bit integer value from the I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	@SuppressWarnings("unused")
	private int readUInt16(int address)
	{
		byte[] buffer = new byte[2];
		readI2C(address, buffer);
		return ((buffer[1] & 0xff) << 8) | (buffer[0] & 0xff);
	}
	
	/**
	 * Reads an array of un-signed 16-bit integer values from the I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	@SuppressWarnings("unused")
	private void readUInt16s(int address, int[] data)
	{
		byte[] buffer = new byte[data.length * 2];
		readI2C(address, buffer);
		for (int i = 0; i < data.length; i++)
		{
			data[i] = ((buffer[i * 2 + 1] & 0xff) << 8) | (buffer[i * 2] & 0xff);
		}
	}
	
	/**
	 * Resets the device.
	 * @return true if reset successful
	 */
	private boolean resetDevice()
	{
		int status;
		
		// Reset the device:
		writeI2C(RESET_REQ_REGISTER, 0b00000001);
		
		// Make sure the EEPROM was detected:
		boolean eepromDetected = false;
		for (int i = 0; i < 30 && !eepromDetected; i++)
		{
			try
			{
				Thread.sleep(100);
			}
			catch (Exception ex)
			{
				// ignore
			}
			status = readByte(SENTRAL_STATUS_REGISTER);
			eepromDetected = (status & SENTRAL_STATUS_EEPROM_DETECTED) != 0;
		}
		
		if (!eepromDetected)
		{
			log("SENtral initialization error: No EEPROM detected");
			return false;
		}

		// Make sure the EEPROM was uploaded
		boolean uploadDone = false;
		for (int i = 0; i < 30 && !uploadDone; i++)
		{
			try
			{
				Thread.sleep(100);
			}
			catch (Exception ex)
			{
				// ignore
			}
			status = readByte(SENTRAL_STATUS_REGISTER);
			uploadDone = (status & SENTRAL_STATUS_EEPROM_UPLOAD_COMPLETED) != 0;
		}
		
		if (!uploadDone)
		{
			log("SENtral initialization error: EEPROM upload did not complete");
			return false;
		}
		
		// Check for EEPROM upload error:
		status = readByte(SENTRAL_STATUS_REGISTER);
		if ((status & SENTRAL_STATUS_EEPROM_EEPROM_UPLOAD_ERROR) != 0)
		{
			log("SENtral initialization error: Bad EEPROM CRC");
			return false;
		}
		
		return true;
	}
	
	/**
	 * Enables the sensors and the SENtral algorithm.
	 */
	private void run()
	{
		// Start the sample rate calculators:
		magSampleRate.start();
		accelSampleRate.start();
		gyroSampleRate.start();
		hprSampleRate.start();
		
		// Enable processing data from the SENtral:
		enableDataProcessing();
		
		// Turn "on" the device:
		writeI2C(HOST_CONTROL_REGISTER, HOST_CONTROL_RUN_ENABLE);
	}
	
	/**
	 * Sets the device algorithm control.
	 * @return true if algorithm control set successfully;
	 */
	private boolean setAlgorithmControl()
	{
		writeI2C(ALGORITHM_CONTROL_REGISTER, ALGORITHM_CONTROL_HPR_OUTPUT);
		return true;
	}
	
	/**
	 * Sets the events enabled from the device;
	 * @return true if operation was successful
	 */
	private boolean setEnabledEvents()
	{
		if (interrupt != null)
		{
			writeI2C(ENABLE_EVENTS_REGISTER, ENABLE_EVENTS_ALL);
		}
		return true;
	}
	
	private boolean setSampleRates()
	{
		byte[] rates =  { MAG_RATE, ACCEL_RATE, GYRO_RATE } ;
		writeI2C(MAG_RATE_REGISTER, rates);
		writeI2C(Q_RATE_DIVISOR_REGISTER, Q_RATE_DIVISOR);
		
		byte[] buffer = new byte[rates.length];
		readI2C(MAG_RATE_REGISTER, buffer);
		return true;
	}
	
	/**
	 * Writes a single byte of data to the specified I2C device at the specified register address.
	 */
	private void writeI2C(int address, int data)
	{
		if (i2c.write(address, data))
		{
			log(String.format("Failed to write %02X to 0x%02X", data, address));
		}
	}
	
	/**
	 * Writes multiple bytes of data to the specified I2C device at the specified register address, and subsequent addresses.
	 */
	private void writeI2C(int address, byte[] buffer)
	{
		byte[] newBuffer = new byte[buffer.length + 1];
		newBuffer[0] = (byte)address;
		System.arraycopy(buffer, 0, newBuffer, 1, buffer.length);
		
		if (i2c.writeBulk(newBuffer))
		{
			log(String.format("Failed to write %d bytes to 0x%02X", buffer.length, address));
		}
	}
}
