package com.momentum4999.robot.triggers;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class UndervoltageTrigger extends Trigger {
	private static final double TRIGGER_VOLT = 8;
	private static final double TRIGGER_TIME = 0.5;
	private PowerDistribution pdp;

	private long brownoutStart = System.currentTimeMillis();

	public UndervoltageTrigger(PowerDistribution pdp) {
		this.pdp = pdp;
	}

	@Override
	public boolean get() {
		if(pdp.getVoltage() < TRIGGER_VOLT) {
			return System.currentTimeMillis() - brownoutStart > (TRIGGER_TIME * 1000);
		} else {
			brownoutStart = System.currentTimeMillis();
			return false;
		}
	}
}
