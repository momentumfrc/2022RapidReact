package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroClimberCommand extends CommandBase {
	private final static double ZEROING_POWER = 0.3;

	private final ClimberSubsystem climber;

	public ZeroClimberCommand(ClimberSubsystem climber, PowerDistribution pdp) {
		this.climber = climber;
		addRequirements(climber);
	}

	@Override
	public void execute() {
		climber.leftClimber.zero(ZEROING_POWER);
		climber.rightClimber.zero(ZEROING_POWER);
	}

	@Override
	public boolean isFinished() {
		return climber.leftClimber.getHasZero() && climber.rightClimber.getHasZero();
	}
}
