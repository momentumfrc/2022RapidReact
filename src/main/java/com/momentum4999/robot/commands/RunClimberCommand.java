package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.subsystems.ClimberSubsystem;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimberCommand extends CommandBase {
	private ClimberSubsystem climber;
	private MoInput input;

	private static final double CLIMBER_DEADZONE = 0.05;

	public RunClimberCommand(ClimberSubsystem climber, MoInput input ) {
		this.climber = climber;
		this.input = input;

		addRequirements(climber);
	}

	@Override
	public void execute() {
		double leftClimbRequest = input.getElevatorLeft();
		double rightClimbRequest = input.getElevatorRight();

		climber.leftClimber.raise(leftClimbRequest);
		climber.rightClimber.raise(rightClimbRequest);
	}

	@Override
	public void end(boolean interrupted) {
		this.climber.stop();
	}
}
