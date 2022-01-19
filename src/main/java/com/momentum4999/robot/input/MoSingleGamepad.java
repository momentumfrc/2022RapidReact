package com.momentum4999.robot.input;

import edu.wpi.first.wpilibj.GenericHID;

public class MoSingleGamepad extends InputDevice {
	private final GenericHID gamepad;

	public MoSingleGamepad(GenericHID gamepad) {
		this.gamepad = gamepad;
	}

	@Override
	public double axisStatusInternal(int axis) {
		return gamepad.getRawAxis(axis);
	}

	@Override
	protected boolean buttonStatusInternal(int button) {
		return gamepad.getRawButton(button);
	}
}
