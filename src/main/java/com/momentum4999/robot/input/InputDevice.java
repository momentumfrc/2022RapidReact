package com.momentum4999.robot.input;

import java.util.List;
import java.util.function.Consumer;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class InputDevice {
	public abstract double axisStatusInternal(InputAxis axis);

	protected abstract boolean buttonStatusInternal(InputButton button);

	public abstract JoystickButtonHolder getJoystickButton(InputButton button);

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
		LX(0, -1, 1), LY(1, -1, 1), 
		RX(4, -1, 1), RY(5, -1, 1),
		LT(2, 0, 1), RT(3, 0, 1);

		public final int id;
		private final double min;
		private final double max;

		InputAxis(int id, double min, double max) {
			this.id = id;
			this.min = min;
			this.max = max;
		}
	}

	public class JoystickButtonHolder {
		private final List<JoystickButton> buttons;
		
		public JoystickButtonHolder(List<JoystickButton> buttons) {
			this.buttons = buttons;
		}

		public void apply(Consumer<JoystickButton> action) {
			this.buttons.forEach(action);
		}
	}
}
