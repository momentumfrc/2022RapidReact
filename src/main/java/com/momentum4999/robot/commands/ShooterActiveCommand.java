package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterActiveCommand extends Command {
	private final ShooterSubsystem shooter;

	public ShooterActiveCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;

		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.runActive();
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}
}
