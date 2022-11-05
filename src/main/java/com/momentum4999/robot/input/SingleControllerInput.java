package com.momentum4999.robot.input;

import org.usfirst.frc.team4999.controllers.LogitechF310;
import org.usfirst.frc.team4999.utils.Utils;

public class SingleControllerInput implements MoInput {

	private static final double DEADZONE = 0.05;

	private LogitechF310 f310;

	public SingleControllerInput(LogitechF310 f310) {
		this.f310 = f310;
	}

	@Override
	public double getMoveRequest() {
		return -1 * Utils.deadzone(f310.getRawAxis(1), DEADZONE);
	}

	@Override
	public double getTurnRequest() {
		return Utils.deadzone(f310.getRawAxis(4), DEADZONE);
	}

	@Override
	public boolean getRunShooter() {
		return f310.getBButton();
	}

	@Override
	public double getElevatorLeft() {
		return -1 * Utils.deadzone(f310.getRawAxis(5) * f310.getRawAxis(2), DEADZONE);
	}

	@Override
	public double getElevatorRight() {
		return -1 * Utils.deadzone(f310.getRawAxis(5) * f310.getRawAxis(3), DEADZONE);
	}
}
