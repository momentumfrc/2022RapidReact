package com.momentum4999.robot.util;

import org.usfirst.frc.team4999.utils.Utils;

/**
 * A class for keeping track of motor power and allowing for power
 * to change over time
 */
public class MotorRamp {
	private final double ramp;

	private double currentPower = 0;

	/**
	 * Constructs a {@link MotorRamp}
	 * 
	 * @param ramp the amount of power added to the motors per update
	 */
	public MotorRamp(double ramp) {
		this.ramp = ramp;
	}

	/**
	 * Updates this motor ramp to run at a specified power
	 * 
	 * @param power the power that this motor ramp should adjust towards
	 */
	public void run(double power) {
		if (Math.abs(power - this.currentPower) <= this.ramp) {
			this.currentPower = power;
		}

		double inc = 0;
		if (power > this.currentPower) {
			inc = this.ramp;
		} else if (power < this.currentPower) {
			inc = -this.ramp;
		}

		this.currentPower = Utils.clip(this.currentPower + inc, -1, 1);
	}

	/**
	 * Gets the current power this motor ramp is running at
	 * 
	 * @return the current power of this motor ramp
	 */
	public double power() {
		return this.currentPower;
	}
}
