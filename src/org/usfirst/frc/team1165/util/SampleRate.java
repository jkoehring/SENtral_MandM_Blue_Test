package org.usfirst.frc.team1165.util;

/**
 * Class for computing sample rates based upon a rolling average.
 *
 */
public class SampleRate
{
	private Rolling rolling;
	private long startTime;
	private int rollingWindowSize;
	
	public SampleRate()
	{
		this(100);
	}
	
	public SampleRate(int rollingWindowSize)
	{
		this.rollingWindowSize = rollingWindowSize;
	}
	
	/**
	 * Adds a sample.
	 */
	public void addSample()
	{
		synchronized(this)
		{
			long now = System.nanoTime();
			rolling.add(now - startTime);
			startTime = now;
		}
	}
	
	/**
	 * Starts computing sample rates now.
	 */
	public void start()
	{
		synchronized(this)
		{
			startTime = System.nanoTime();
			rolling = new Rolling(rollingWindowSize);
		}
	}
	
	/**
	 * Returns sample rate in samples per second.
	 */
	public double getSampleRate()
	{
		synchronized(this)
		{
			return 1e9 / rolling.getAverage();
		}
	}
}
