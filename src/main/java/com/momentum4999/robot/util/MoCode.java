package com.momentum4999.robot.util;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
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
 * <p>turn left 1 second=
 * 
 */
public class MoCode {
	public static final MoCode INSTANCE = new MoCode()
		.withStep("wait", WaitStep::new)
		.withStep("drive", DriveStep::new)
		.withStep("turn", TurnStep::new)
		.withStep("set", SetPowerStep::new)
		.withStep("intake", IntakeStep::new)
		.withStep("shoot", ShootStep::new);
	
	private final Map<String, Function<String[], Step>> steps = new HashMap<>();

	private final Map<String, MoCodeRuntime> loadedScripts = new HashMap<>();
	private String defaultScript;

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

	public void loadScripts(RobotContainer robot) {
		this.loadedScripts.clear();
		this.defaultScript = null;

		String scriptList = MoUtil.readResourceFile("scripts.txt");

		for (String scriptName : scriptList.split("\n")) {
			String scriptSrc = MoUtil.readResourceFile("scripts/"+scriptName);
			this.loadedScripts.put(scriptName, this.compile(robot, scriptSrc));

			if (this.defaultScript == null) {
				this.defaultScript = scriptName;
			}
		}
	}

	/**
	 * Compiles MoCode source code into a runnable function.
	 * 
	 * <p>The resulting {@link MoCodeRuntime} must be updated periodically to run its contained command steps.
	 * 
	 * @param source The MoCode source code
	 * @return a runtime that can run the compiled code
	 */
	public MoCodeRuntime compile(RobotContainer robot, String source) {
		String[] lines = source.split("\n");
		
		List<Step> compiledSteps = new ArrayList<>();

		for (String line : lines) {
			String[] tokens = line.split("\\s");

			if (tokens.length <= 0 || tokens[0] == null) continue;

			String stepType = tokens[0];

			if (this.steps.containsKey(stepType)) {
				compiledSteps.add(this.steps.get(stepType).apply(tokens));
			}
		}

		return new MoCodeRuntime(robot, compiledSteps);
	}

	public Set<String> getLoadedScripts() {
		return this.loadedScripts.keySet();
	}

	public MoCodeRuntime getRunnableScript(String library) {
		return this.loadedScripts.get(library);
	}

	public String getDefaultScriptName() {
		return this.defaultScript;
	}

	public static class MoCodeRuntime {
		public final RobotContainer robot;
		public double drivePower = 1;
		public int intakeRunning = 0;

		private final Deque<Step> toExecute;
		private long lastCommandTimestamp = -1;

		public MoCodeRuntime(RobotContainer robot, Collection<Step> steps) {
			this.robot = robot;
			this.toExecute = new ArrayDeque<>(steps);
		}

		public void markAsRunningNewCommand() {
			this.lastCommandTimestamp = System.currentTimeMillis();
		}

		public long commandRunTime() {
			return System.currentTimeMillis() - this.lastCommandTimestamp;
		}

		public void beginIntaking(boolean rev) {
			this.robot.beginIntake();
			this.intakeRunning = rev ? -1 : 1;
		}

		public void endIntaking() {
			this.intakeRunning = 0;
			this.robot.endIntake();
		}

		public void periodic() {
			if (this.toExecute.size() > 0) {
				if (lastCommandTimestamp <= 0) {
					this.toExecute.peekFirst().init(this);
				}

				// Run the current command and check if terminated
				boolean run = this.toExecute.peekFirst().execute(this);
								
				if (!run) {
					// If terminated, remove running command
					this.toExecute.removeFirst();
					// Init the next command
					if (this.toExecute.size() > 0) {
						this.toExecute.peekFirst().init(this);
					}
				}

				this.updateOthers();
			} else {
				this.robot.stopSubsystems();
			}
		}

		public void updateOthers() {
			if (intakeRunning != 0) {
				this.robot.runIntake(intakeRunning < 0);
			}
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
		public void init(MoCodeRuntime runtime) {
			runtime.markAsRunningNewCommand();
		}

		public abstract boolean execute(MoCodeRuntime runtime);
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
		public boolean execute(MoCodeRuntime runtime) {
			int timeMillis = (int) (this.waitTime * 1000);

			if (runtime.commandRunTime() < timeMillis) {
				return true;
			}
			return false;
		}
	}

	public static class DriveStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("forward", "backward").number().literal("second", "seconds", "meter", "meters");
		public final double time;
		public final double distance;
		public final double speed;

		private double startDistance;

		public DriveStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				if (tokens[3].startsWith("meter")) {
					this.distance = Double.parseDouble(tokens[2]);
					this.time = 0;
				} else {
					this.time = Double.parseDouble(tokens[2]);
					this.distance = 0;
				}
				this.speed = "backward".equals(tokens[1]) ? -1 : 1;
			} else {
				this.speed = 0;
				this.time = 0;
				this.distance = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void init(MoCodeRuntime runtime) {
			super.init(runtime);

			this.startDistance = runtime.robot.driveSubsystem.getAverageDrivenDistance();
		}

		@Override
		public boolean execute(MoCodeRuntime runtime) {
			double runSpeed = this.speed * runtime.drivePower;

			if (this.distance > 0) {
				double distanceDiff = Math.abs(this.startDistance - runtime.robot.driveSubsystem.getAverageDrivenDistance());
				if (distanceDiff < this.distance) {
					runtime.robot.driveSubsystem.driveDirect(runSpeed, runSpeed);

					return true;
				}
			} else {
				int timeMillis = (int) (this.time * 1000);

				if (runtime.commandRunTime() < timeMillis) {
					runtime.robot.driveSubsystem.driveDirect(runSpeed, runSpeed);

					return true;
				}
			}
			runtime.robot.driveSubsystem.driveDirect(0, 0);

			return false;
		}
	}

	public static class TurnStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("left", "right").number().literal("second", "seconds", "degree", "degrees");
		public final double time;
		public final boolean left;
		public final double angle;

		private double startAngle;

		public TurnStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				if (tokens[3].startsWith("degree")) {
					this.angle = Double.parseDouble(tokens[2]);
					this.time = 0;
				} else {
					this.time = Double.parseDouble(tokens[2]);
					this.angle = 0;
				}
				this.left = "left".equals(tokens[1]);
			} else {
				this.left = false;
				this.time = 0;
				this.angle = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void init(MoCodeRuntime runtime) {
			super.init(runtime);

			this.startAngle = runtime.robot.driveSubsystem.getPose().getRotation().getDegrees();
		}

		@Override
		public boolean execute(MoCodeRuntime runtime) {
			double runSpeed = (this.left ? -1 : 1) * runtime.drivePower;

			if (this.angle > 0) {
				double angleDiff = Math.abs(MoUtil.wrapAngleDeg(this.startAngle - runtime.robot.driveSubsystem.getPose().getRotation().getDegrees()));

				runtime.robot.driveSubsystem.driveDirect(runSpeed, -runSpeed);
				if (angleDiff < this.angle) {
					return true;
				}
			} else {
				int timeMillis = (int) (this.time * 1000);

				if (runtime.commandRunTime() < timeMillis) {
					runtime.robot.driveSubsystem.driveDirect(runSpeed, -runSpeed);

					return true;
				}
			}
			runtime.robot.driveSubsystem.driveDirect(0, 0);
			
			return false;
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
		public boolean execute(MoCodeRuntime runtime) {
			runtime.drivePower = this.power;

			return false;
		}
	}

	public static class IntakeStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.literal("forward", "reverse", "stop");
		public final boolean start;
		public final boolean rev;
		private final boolean valid;

		public IntakeStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.start = !("stop".equals(tokens[1]));
				this.rev = "reverse".equals(tokens[1]);
				this.valid = true;
			} else {
				this.start = false;
				this.rev = false;
				this.valid = false;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public void init(MoCodeRuntime runtime) {
			super.init(runtime);

			runtime.robot.beginIntake();
		}

		@Override
		public boolean execute(MoCodeRuntime runtime) {
			if (this.valid) {
				if (this.start) {
					runtime.beginIntaking(this.rev);
				} else {
					runtime.endIntaking();
				}
			}

			return false;
		}
	}

	public static class ShootStep extends Step {
		private static final LineValidator VALIDATOR = new LineValidator()
			.number().literal("second", "seconds");
		public final double time;

		public ShootStep(String[] tokens) {
			if (VALIDATOR.validate(tokens)) {
				this.time = Double.parseDouble(tokens[1]);
			} else {
				this.time = 0;
				System.out.println("Invalid Line: "+String.join(" ", tokens));
			}
		}

		@Override
		public boolean execute(MoCodeRuntime runtime) {
			int timeMillis = (int) (this.time * 1000);

			if (runtime.commandRunTime() < timeMillis) {
				runtime.robot.shoot();

				return true;
			}
			runtime.robot.stopShooting();

			return false;
		}
	}
}
