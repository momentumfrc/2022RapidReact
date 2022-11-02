package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoBaseInput;
import com.momentum4999.robot.input.InputMapping.InputAxis;
import com.momentum4999.robot.subsystems.ClimberSubsystem;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimberCommand extends CommandBase {
	private ClimberSubsystem climber;
	private MoBaseInput input;

	private static final double CLIMBER_DEADZONE = 0.05;

	public RunClimberCommand(ClimberSubsystem climber, MoBaseInput input) {
		this.climber = climber;
		this.input = input;

		addRequirements(climber);
	}

	@Override
	public void execute() {
		double leftTrigger = input.getAxis(InputAxis.LT);
		double rightTrigger = input.getAxis(InputAxis.RT);


		leftTrigger = Utils.clip(Utils.map(leftTrigger, CLIMBER_DEADZONE, 1, 0, 1), 0, 1);
		rightTrigger = Utils.clip(Utils.map(rightTrigger, CLIMBER_DEADZONE, 1, 0, 1), 0, 1);
		if(leftTrigger > 0) {
			climber.leftClimber.raise(-input.getAxis(InputAxis.RY) * leftTrigger);
		}

		if(rightTrigger > 0) {
			climber.rightClimber.raise(-input.getAxis(InputAxis.RY) * rightTrigger);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.climber.stop();
	}
}
