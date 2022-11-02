package com.momentum4999.robot.triggers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StallTrigger extends Trigger {
	public static interface StallTriggerSource {
		public double getMotorSetPower();
		public double getMotorRPM();
	}

	public static class StallTriggerOptions {
		public final double cutOffPwr;
		public final double cutOffRpm;
		public final double cutOffTime;

		public StallTriggerOptions(double cutOffPwr, double cutOffRpm, double cutOffTime) {
			this.cutOffPwr = cutOffPwr;
			this.cutOffTime = cutOffTime;
			this.cutOffRpm = cutOffRpm;
		}

		public static StallTriggerOptions defaultOptions() {
			return new StallTriggerOptions(
				0.05,
				1,
				0.5
			);
		}
	}

	private final StallTriggerSource source;
	private final StallTriggerOptions options;
	private long stallStartTs = System.currentTimeMillis();

	public static StallTrigger makeForSparkMax(CANSparkMax max, StallTriggerOptions options) {
		var source = new StallTriggerSource() {
			@Override
			public double getMotorSetPower() {
				return max.get();
			}

			@Override
			public double getMotorRPM() {
				var encoder = max.getEncoder();
				return encoder.getVelocity() / encoder.getVelocityConversionFactor();
			}
		};

		return new StallTrigger(source, options);
	}

	public static StallTrigger makeForSparkMax(CANSparkMax max) {
		return StallTrigger.makeForSparkMax(max, StallTriggerOptions.defaultOptions());
	}

	public StallTrigger(StallTriggerSource source, StallTriggerOptions options) {
		this.source = source;
		this.options = options;
	}

	@Override
	public boolean get() {
		boolean isStalled = Math.abs(source.getMotorRPM()) <= options.cutOffRpm && Math.abs(source.getMotorSetPower()) >= options.cutOffPwr;
		if(isStalled) {
			return ((System.currentTimeMillis() - stallStartTs) / 1000.0) > options.cutOffTime;
		} else {
			stallStartTs = System.currentTimeMillis();
			return false;
		}
	}


}
