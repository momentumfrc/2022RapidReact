package com.momentum4999.robot.input;

public abstract class InputMapping {

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
		public final double min;
		public final double max;

		InputAxis(int id, double min, double max) {
			this.id = id;
			this.min = min;
			this.max = max;
		}
	}

	public static class ButtonMapping {
	}
}
