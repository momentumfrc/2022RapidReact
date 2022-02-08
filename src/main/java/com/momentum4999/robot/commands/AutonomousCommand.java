package com.momentum4999.robot.commands;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;

import com.momentum4999.robot.RobotContainer;
import com.momentum4999.robot.util.MoCode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
	private static ExecutorService EXECUTOR = Executors.newSingleThreadExecutor();

	private final RobotContainer robot;
	private final Consumer<RobotContainer> procedure;

	public AutonomousCommand(RobotContainer robot, Consumer<RobotContainer> procedure) {
		this.robot = robot;
		this.procedure = procedure;
	}

	public AutonomousCommand(RobotContainer robot, String moCodeSource) {
		this(robot, MoCode.INSTANCE.compile(moCodeSource));
	}

	@Override
	public void initialize() {
		EXECUTOR.submit(() -> this.procedure.accept(this.robot));
	}

	@Override
	public void end(boolean interrupted) {
		EXECUTOR.shutdownNow();
		EXECUTOR = Executors.newSingleThreadExecutor();
		this.robot.stopSubsystems();
	}
}
