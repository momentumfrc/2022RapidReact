package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.subsystems.TargetingSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterActiveCommand extends CommandBase {
	private final ShooterSubsystem shooter;
	private final TargetingSubsystem targeting;

	private final boolean doTargeting;

	public ShooterActiveCommand(ShooterSubsystem shooter, TargetingSubsystem targeting, boolean doTargeting) {
		this.shooter = shooter;
		this.targeting = targeting;

		this.doTargeting = doTargeting;

		addRequirements(shooter, targeting);
	}

	@Override
	public void initialize() {
		targeting.limelight.setLight(true);
	}

	@Override
	public void execute() {
		shooter.runActive(doTargeting);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
		targeting.limelight.setLight(false);
	}
}
