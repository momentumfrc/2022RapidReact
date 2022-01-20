package com.momentum4999.robot.input;

import edu.wpi.first.wpilibj.GenericHID;

public class MoSingleGamepad extends InputDevice {
	private final GenericHID gamepad;

	public MoSingleGamepad(GenericHID gamepad) {
		this.gamepad = gamepad;
	}

	@Override
	public double axisStatusInternal(InputAxis axis) {
		return gamepad.getRawAxis(axis.id);
	}

	@Override
	protected boolean buttonStatusInternal(InputButton button) {
		return gamepad.getRawButton(button.id);
	}
}
