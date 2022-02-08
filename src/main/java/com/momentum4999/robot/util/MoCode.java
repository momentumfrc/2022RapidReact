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
		.withStep("turn", TurnStep::new);
	
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
	public Consumer<RobotContainer> compile(String source) {
		String[] lines = source.split("\n");
		
		List<Consumer<RobotContainer>> procedures = new ArrayList<>();

		for (String line : lines) {
			String[] tokens = line.split("\\s");

			if (tokens.length == 0) continue;

			String stepType = tokens[0];
			Step step = this.steps.get(stepType).apply(tokens);

			procedures.add(step::execute);
		}

		return robot -> {
			procedures.forEach(p -> p.accept(robot));
		};
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
		public abstract void execute(RobotContainer robot);
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
		public void execute(RobotContainer robot) {
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
		public void execute(RobotContainer robot) {
			try {
				robot.driveSubsystem.driveDirect(this.speed, this.speed);
				Thread.sleep((long)(this.time * 1000));
				robot.driveSubsystem.stop();
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
		public void execute(RobotContainer robot) {
			try {
				robot.driveSubsystem.driveDirect(this.speed, -this.speed);
				Thread.sleep((long)(this.time * 1000));
				robot.driveSubsystem.stop();
			} catch (InterruptedException ignored) {}
		}
	}
}
