package com.momentum4999.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public final class MoUtil {
	private MoUtil() {}

	public static DoubleSolenoid.Value getSolenoidOpposite(DoubleSolenoid.Value value) {
		switch (value) {
			case kForward: return DoubleSolenoid.Value.kReverse;
			case kReverse: return DoubleSolenoid.Value.kForward;
			default:       return DoubleSolenoid.Value.kOff;
		}
	}
}
