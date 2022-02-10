package com.momentum4999.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;

import com.momentum4999.robot.RobotContainer;

import org.usfirst.frc.team4999.utils.Utils;

/**
 * A compiler for MoCode, which can be used to write very 
 * simple robot procedures in a human readable format.
 * 
 * <p>The language is designed so that individuals with no experience
 * using the language can both read and make edits to code written
 * in it.
 * 
 * <p>An example of MoCode:
 * 
 * <p>wait 1 second
 * <p>drive forward 3 seconds
 * <p>wait 0.5 seconds
 * <p>turn left 1 second
 */
public class MoCode {
	public static final MoCode INSTANCE = new MoCode()
		.withStep("wait", WaitStep::new)
		.withStep("drive", DriveStep::new)
		.withStep("turn", TurnStep::new)
		.withStep("set", SetPowerStep::new);
	
	private final Map<String, Function<String[], Step>> steps = new HashMap<>();

	/**
	 * Add a type of MoCode step to this compiler
	 * 
	 * @param name The name of the step to be used in MoCode
	 * @param step A {@link Step} constructor
	 * @return this MoCode instance to allow more method chaining
	 */
	public MoCode withStep(String name, Function<String[], Step> step) {
		this.steps.put(name, step);

		return this;
	}

	/**
	 * Compiles MoCode source code into a runnable function.
	 * 
	 * <p>The resulting {@link Consumer} <em>must be run on a separate thread from normal execution.</em>
	 * 
	 * @param source The MoCode source code
	 * @return a function which runs the compiled code
	 */
	public Consumer<MoCodeRuntime> compile(String source) {
		String[] lines = source.split("\n");
		
		List<Consumer<MoCodeRuntime>> procedures = new ArrayList<>();

		for (String line : lines) {
			String[] tokens = line.split("\\s");

			if (tokens.length <= 0 || tokens[0] == null) continue;

			String stepType = tokens[0];

			if (this.steps.containsKey(stepType)) {
				Step step = this.steps.get(stepType).apply(tokens);

				procedures.add(step::execute);
			}
		}

		return robot -> {
			procedures.forEach(p -> p.accept(robot));
		};
	}

	public static class MoCodeRuntime {
		public final RobotContainer robot;
		public double drivePower = 1;
		public double driveLeft = 0;
		public double driveRight = 0;

		public MoCodeRuntime(RobotContainer robot) {
			this.robot = robot;
		}

		public void setRobotDrive(double left, double right) {
			this.driveLeft = left;
			this.driveRight = right;
		}

		public void periodic() {
			this.robot.driveSubsystem.driveDirect(this.driveLeft, this.driveRight);
		}
	}

	/**
	 * Validates an array of source code tokens against a set of expectations
	 */
	public static class LineValidator {
		private final List<Predicate<String>> tokenValidators = new ArrayList<>();
		
		public LineValidator() {}

		public LineValidator literal(String ... choices) {
			List<String> choiceList = Arrays.asList(choices);
			this.tokenValidators.add(s -> choiceList.contains(s));

			return this;
		}

		public LineValidator number() {
			this.tokenValidators.add(s -> s.matches("[+-]?\\d*\\.?\\d*?"));

			return this;
		}

		public boolean validate(String[] tokens) {
			if ((tokens.length - 1) < tokenValidators.size()) {
				return false;
			}

			for (int i = 0; i < tokenValidators.size(); i++) {
				if (!tokenValidators.get(i).test(tokens[i + 1])) {
					return false;
				}
			}
			return true;
		}
	}

	/**
	 * The base class for a MoCode step type
	 */
	public static abstract class Step {
		public abstract void execute(MoCodeRuntime runtime);
	}

	public static class WaitStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.number().literal("second", "seconds");
		public final double waitTime;

		public WaitStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.waitTime = Double.parseDouble(tokens[1]);
			} else {
				this.waitTime = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void execute(MoCodeRuntime runtime) {
			try {
				Thread.sleep((long)(this.waitTime * 1000));
			} catch (InterruptedException ignored) {}
		}
	}

	public static class DriveStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("forward", "backward").number().literal("second", "seconds");
		public final double time;
		public final double speed;

		public DriveStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.speed = "backward".equals(tokens[1]) ? -1 : 1;
				this.time = Double.parseDouble(tokens[2]);
			} else {
				this.speed = 0;
				this.time = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void execute(MoCodeRuntime runtime) {
			System.out.println("DRIVE");
			try {
				double speed = this.speed * runtime.drivePower;

				runtime.setRobotDrive(speed, speed);
				Thread.sleep((long)(this.time * 1000));
				runtime.setRobotDrive(0, 0);
			} catch (InterruptedException ignored) {}
		}
	}

	public static class TurnStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("left", "right").number().literal("second", "seconds");
		public final double time;
		public final double speed;

		public TurnStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.speed = "left".equals(tokens[1]) ? -1 : 1;
				this.time = Double.parseDouble(tokens[2]);
			} else {
				this.speed = 0;
				this.time = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void execute(MoCodeRuntime runtime) {
			try {
				double speed = this.speed * runtime.drivePower;

				runtime.setRobotDrive(speed, -speed);
				Thread.sleep((long)(this.time * 1000));
				runtime.setRobotDrive(0, 0);
			} catch (InterruptedException ignored) {}
		}
	}

	public static class SetPowerStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("drive").literal("power").number().literal("percent");
		public final double power;

		public SetPowerStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.power = Utils.clip(Double.parseDouble(tokens[3]), 0, 100) * 0.01;
			} else {
				this.power = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void execute(MoCodeRuntime runtime) {
			runtime.drivePower = this.power;
		}
	}
}
