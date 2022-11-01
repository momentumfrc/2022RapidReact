package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoBaseInput;
import com.momentum4999.robot.input.InputMapping.InputAxis;
import com.momentum4999.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimberCommand extends CommandBase {
	private ClimberSubsystem climber;
	private MoBaseInput input;

	public RunClimberCommand(ClimberSubsystem climber, MoBaseInput input) {
		this.climber = climber;
		this.input = input;

		addRequirements(climber);
	}

	@Override
	public void execute() {
		if (this.input.getAxis(InputAxis.LT) > 0.05 || this.input.getAxis(InputAxis.RT) > 0.05) {
			this.climber.raiseLeft(-input.getAxis(InputAxis.LY) * input.getAxis(InputAxis.LT));
			this.climber.raiseRight(-input.getAxis(InputAxis.LY) * input.getAxis(InputAxis.RT));
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.climber.stop();
	}
}
