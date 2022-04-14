package com.momentum4999.robot.input;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Predicate;

import com.momentum4999.robot.input.InputMapping.InputAxis;
import com.momentum4999.robot.input.InputMapping.InputButton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class MoMultiInput extends MoBaseInput {
	private final List<Entry> gamepads;

	public MoMultiInput(Entry entry, Entry ... others) {
		List<Entry> entries = new ArrayList<>(Arrays.asList(others));
		entries.add(0, entry);

		this.gamepads = entries;
	}

	@Override
	public double axisStatusInternal(InputAxis axis) {
		double value = 0;

		for (Entry e : gamepads) {
			if (e.axisFilter.test(axis)) {
				double axisValue = e.gamepad.getRawAxis(axis.id);
				if (Math.abs(axisValue) > Math.abs(value)) {
					value = axisValue;
				}
			}
		}

		return value;
	}

	@Override
	protected boolean buttonStatusInternal(InputButton button) {
		for (Entry e : gamepads) {
			if (e.buttonFilter.test(button)) {
				return e.gamepad.getRawButton(button.id);
			}
		}

		return false;
	}

	@Override
	public JoystickButtonHolder getJoystickButton(InputButton button) {
		List<JoystickButton> buttons = new ArrayList<>();
		for (Entry e : gamepads) {
			if (e.buttonFilter.test(button)) {
				buttons.add(new JoystickButton(e.gamepad, button.id));
			}
		}

		return new JoystickButtonHolder(buttons);
	}

	public static Entry entry(GenericHID controller) {
		return new Entry(controller);
	}

	public static class Entry {
		public final GenericHID gamepad;
		private Predicate<InputButton> buttonFilter = b -> true;
		private Predicate<InputAxis> axisFilter = a -> true;

		public Entry(GenericHID gamepad) {
			this.gamepad = gamepad;
		}

		public Entry denyButtons(InputButton ... buttons) {
			List<InputButton> buttonList = Arrays.asList(buttons);
			this.buttonFilter = this.buttonFilter.and(buttonList::contains);

			return this;
		}

		public Entry denyAxes(InputAxis ... axes) {
			List<InputAxis> axisList = new ArrayList<>(Arrays.asList(axes));
			this.axisFilter = this.axisFilter.and(axisList::contains);

			return this;
		}
	}
}
