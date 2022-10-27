package com.momentum4999.robot.triggers;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OvercurrentTrigger extends Trigger {

	private double currentLimit;
	private double cutoffTime; // in seconds
	private DoubleSupplier currentSupplier;

	long overCurrentStartTs = 0;

	public static OvercurrentTrigger makeForSparkMax(double currentLimit, double cutoffTime, CANSparkMax spark) {
		return new OvercurrentTrigger(currentLimit, cutoffTime, () -> spark.getOutputCurrent());
	}

	public static OvercurrentTrigger makeForPdp(double currentLimit, double cutoffTime, PowerDistribution pdp, int pdpChannel) {
		return new OvercurrentTrigger(currentLimit, cutoffTime, () -> pdp.getCurrent(pdpChannel));
	}

	public OvercurrentTrigger(double currentLimit, double cutoffTime, DoubleSupplier currentSupplier) {
		this.currentLimit = currentLimit;
		this.cutoffTime = cutoffTime;
		this.currentSupplier = currentSupplier;
	}

	public void setCurrentLimit(double currentLimit) {
		this.currentLimit = currentLimit;
	}

	public void setCutoffTime(double cutoffTime) {
		this.cutoffTime = cutoffTime;
	}

	@Override
	public boolean get() {
		boolean isOverCurrent = currentSupplier.getAsDouble() > currentLimit;
		if(isOverCurrent) {
			return ((System.currentTimeMillis() - overCurrentStartTs) / 1000.0) > cutoffTime;
		} else {
			overCurrentStartTs = System.currentTimeMillis();
			return false;
		}
	}
}
