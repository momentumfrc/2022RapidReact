package com.momentum4999.robot.input;

import org.usfirst.frc.team4999.utils.Utils;

public abstract class InputDevice {
	public abstract double axisStatusInternal(InputAxis axis);

	protected abstract boolean buttonStatusInternal(InputButton button);

	public final boolean getButton(InputButton button) {
		return this.buttonStatusInternal(button);
	}

	public final double getAxis(InputAxis axis) {
		return Utils.clip(this.axisStatusInternal(axis), axis.min, axis.max);
	}

	public enum InputButton {
		X(3), Y(4), A(1), B(2), LB(5), 
		RB(6), START(8), BACK(7);
		
		public final int id;

		InputButton(int id) {
			this.id = id;
		}
	}

	public enum InputAxis {
		LX(2, -1, 1), LY(1, -1, 1), 
		RX(4, -1, 1), RY(5, -1, 1),
		LT(3, -1, 0), RT(3, 0, 1);

		public final int id;
		private final double min;
		private final double max;

		InputAxis(int id, double min, double max) {
			this.id = id;
			this.min = min;
			this.max = max;
		}
	}
}