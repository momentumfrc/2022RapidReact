package com.momentum4999.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class SingleControllerInput implements MoInput {

	private static final double DEADZONE = 0.05;

	private XboxController f310;

	public SingleControllerInput(XboxController f310) {
		this.f310 = f310;
	}

	@Override
	public double getMoveRequest() {
		return -1 * MathUtil.applyDeadband(f310.getRawAxis(1), DEADZONE, 1);
	}

	@Override
	public double getTurnRequest() {
		return MathUtil.applyDeadband(f310.getRawAxis(4), DEADZONE, 1);
	}

	@Override
	public boolean getRunShooter() {
		return f310.getBButton();
	}
}
