package com.momentum4999.robot.input;

import java.util.List;
import java.util.function.Consumer;

import com.momentum4999.robot.input.InputMapping.InputAxis;
import com.momentum4999.robot.input.InputMapping.InputButton;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class MoBaseInput {
	public abstract double axisStatusInternal(InputAxis axis);

	protected abstract boolean buttonStatusInternal(InputButton button);

	public abstract JoystickButtonHolder getJoystickButton(InputButton button);

	public final boolean getButton(InputButton button) {
		return this.buttonStatusInternal(button);
	}

	public final double getAxis(InputAxis axis) {
		return Utils.clip(this.axisStatusInternal(axis), axis.min, axis.max);
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
