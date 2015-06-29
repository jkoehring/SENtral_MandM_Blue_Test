package edu.wpi.first.wpilibj;

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
	 *   [3] Idle: 1 = Device in unprogrammed or Initialized state
	 *   [4] NoEEPROM: 1 = No EEPROM detected
	 */
	public static final int STATUS_REGISTER = 0x37;
	
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
	
	/** AlgorithmStatus Register (read):
	 *   [0] 1 = SENtral in Standby State
	 *       0 = SENtral not in Standby State
	 */
	public static final int ALGORITHM_STATUS_REGISTER = 0x38;
	
	/**
	 * EnableEvents Register:
	 *   '1' indicates in interrupt to the host will be generated for the event
	 *   [0] CPUReset; non-maskable
	 *   [1] Error
	 *   [2] QuaternionResult
	 *   [3] MagResult
	 *   [4] AccelResult
	 *   [5] GyroResult
	 */
	public static final int ENABLE_EVENTS_REGISTER = 0x33;
	
	/* Normal Operation Registers */
	
	/**
	 * HostControl Register (write):
	 *   [0] 1 = Run enable
	 *       0 = Enable initialized state; Standby State generally is preferred
	 *           since enabling Initialized State resets the SENtral algorithm,
	 *           including calibration data
	 */
	public static final int HOST_CONTROL_REGISTER = 0x34;
	
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
	public static final int SENSOR_STATUS_REGISTER = 0x37;
	
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
	 * The I2C object used to communicate.
	 */
	private I2C i2c;
	
	/**
	 * Gyroscope data and access control.
	 */
	private float gyroHeading;
	private float gyroPitch;
	private float gyroRoll;
	private int gyroX;
	private int gyroY;
	private int gyroZ;
	private Object gyroLock = new Object();
	
	/**
	 * Magnetometer data and access control.
	 */
	private int magX;
	private int magY;
	private int magZ;
	private Object magLock = new Object();
	
	/**
	 * Accelerometer data and access control.
	 */
	private int accelX;
	private int accelY;
	private int accelZ;
	private Object accelLock = new Object();
	
	/**
	 * Constructor.
	 */
	public SENtralMandMBlue(Port port, DigitalSource interrupt)
	{
		// Create I2C device:
		i2c = new I2C(port, ADDRESS);
		
		// Register an interrupt handler:
		interrupt.requestInterrupts(new InterruptHandler());

		// Initialize the device:
		initialize(i2c);
	}
	
	/**
	 * Returns X axis acceleration with a range of +/-16 g.
	 */
	public int getAccelX()
	{
		synchronized(accelLock)
		{
			return accelX;
		}
	}
	
	/**
	 * Returns Y axis acceleration with a range of +/-16 g.
	 */
	public int getAccelY()
	{
		synchronized(accelLock)
		{
			return accelY;
		}
	}
	
	/**
	 * Returns Z axis acceleration with a range of +/-16 g.
	 */
	public int getAccelZ()
	{
		synchronized(accelLock)
		{
			return accelZ;
		}
	}
	
	/**
	 * Returns gyroscope heading in radians with a range of +/-Pi.
	 */
	public float getGyroHeading()
	{
		synchronized(gyroLock)
		{
			return gyroHeading;
		}
	}
	
	/**
	 * Returns gyroscope pitch in radians with a range of +/-(Pi/2).
	 */
	public float getGyroPitch()
	{
		synchronized(gyroLock)
		{
			return gyroPitch;
		}
	}
	
	/**
	 * Returns gyroscope roll in radians with a range of +/-Pi.
	 */
	public float getGyroRoll()
	{
		synchronized(gyroLock)
		{
			return gyroRoll;
		}
	}
	
	/**
	 * Returns rotational velocity around X axis with a range of +/-5000 degrees/sec.
	 */
	public int getGyroX()
	{
		synchronized(gyroLock)
		{
			return gyroX;
		}
	}
	
	/**
	 * Returns rotational velocity around Y axis with a range of +/-5000 degrees/sec.
	 */
	public int getGyroY()
	{
		synchronized(gyroLock)
		{
			return gyroY;
		}
	}
	
	/**
	 * Returns rotational velocity around Z axis with a range of +/-5000 degrees/sec.
	 */
	public int getGyroZ()
	{
		synchronized(gyroLock)
		{
			return gyroZ;
		}
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public int getMagX()
	{
		synchronized(magLock)
		{
			return magX;
		}
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public int getMagY()
	{
		synchronized(magLock)
		{
			return magY;
		}
	}
	
	/**
	 * Returns magnetic field about X axis with a range of +/-1000 uT
	 */
	public int getMagZ()
	{
		synchronized(magLock)
		{
			return magZ;
		}
	}
	
	/**
	 * Initialize the device.
	 */
	private void initialize(I2C i2c)
	{
		
	}
	
	/**
	 * Reads a single, unsigned, byte of data from the specified I2C device at the specified register address.
	 */
	private int readByte(I2C i2c, int address)
	{
		byte[] buffer = new byte[1];
		i2c.read(address, buffer.length, buffer);
		return buffer[0] & 0xff;
	}
	
	/**
	 * Reads a signed 16-bit integer value from the specified I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	private int readInt16(I2C i2c, int address)
	{
		byte[] buffer = new byte[2];
		i2c.read(address, buffer.length, buffer);
		return (buffer[1] << 8) | (buffer[0] & 0xff);
	}
	
	/**
	 * Reads an un-signed 16-bit integer value from the specified I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	private int readUInt16(I2C i2c, int address)
	{
		byte[] buffer = new byte[2];
		i2c.read(address, buffer.length, buffer);
		return ((buffer[1] & 0xff) << 8) | (buffer[0] & 0xff);
	}
	
	/**
	 * Reads a 32-bit floating point value from the specified I2C device at the specified register address.
	 * It is assumed that the data retrieved from the I2C device is in little-endian format.
	 */
	private float readFloat32(I2C i2c, int address)
	{
		byte[] buffer = new byte[4];
		i2c.read(address, buffer.length, buffer);
		return Float.intBitsToFloat(((buffer[3] & 0xff) << 24) | ((buffer[2] & 0xff) << 16) | ((buffer[1] & 0xff) << 8) | (buffer[0] & 0xff));
	}
	
	/**
	 * Writes a single byte of data to the specified I2C device at the specified register address.
	 */
	private void write(I2C i2c, int address, int data)
	{
		i2c.write(address, data);
	}
	
	/**
	 * Writes multiple bytes of data to the specified I2C device at the specified register address, and subsequent addresses.
	 */
	private void write(I2C i2c, int address, byte[] buffer)
	{
		i2c.writeBulk(buffer);
	}
	
	/**
	 * Inner class for handling interrupts.
	 */
	private class InterruptHandler<object> extends InterruptHandlerFunction<object>
	{
		@Override
		public void interruptFired(int interruptAssertedMask, Object param)
		{
			
		}
	}
}
