package com.momentum4999.robot.commands;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

import com.momentum4999.robot.Robot;
import com.momentum4999.robot.RobotContainer;
import com.momentum4999.robot.util.MoCode;
import com.momentum4999.robot.util.MoCode.MoCodeRuntime;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
	private static ExecutorService EXECUTOR = Executors.newSingleThreadExecutor();

	private final MoCodeRuntime runtime;
	private final Consumer<MoCodeRuntime> procedure;

	public AutonomousCommand(RobotContainer robot, Consumer<MoCodeRuntime> procedure) {
		this.runtime = new MoCodeRuntime(robot);
		this.procedure = procedure;
	}

	public AutonomousCommand(RobotContainer robot, String moCodeSource) {
		this(robot, MoCode.INSTANCE.compile(moCodeSource));
	}

	@Override
	public void initialize() {
		EXECUTOR.submit(() -> this.procedure.accept(this.runtime));
	}

	@Override
	public void execute() {
		this.runtime.periodic();
	}

	@Override
	public void end(boolean interrupted) {
		EXECUTOR.shutdownNow();
		EXECUTOR = Executors.newSingleThreadExecutor();
		this.runtime.robot.stopSubsystems();
	}

	public static String readAutoMoCodeScript() {
		try (InputStream in = Robot.class.getClassLoader().getResourceAsStream("autonomous.mocod")) {
			if (in == null) {
				System.err.println("Autonomous script file 'autonomous.mocod' doesn't exist!");
				return "";
			}

			return new BufferedReader(new InputStreamReader(in)).lines().reduce((a, b) -> a + "\n" + b).get();
		} catch (IOException ex) {
			System.err.println("Failed to read autonomous script file 'autonomous.mocod': " + ex.getMessage());
		}

		return "";
	}
}
