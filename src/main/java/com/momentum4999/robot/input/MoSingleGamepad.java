package com.momentum4999.robot.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

	@Override
	public JoystickButton getJoystickButton(InputButton button) {
		return new JoystickButton(this.gamepad, button.id);
	}
}
