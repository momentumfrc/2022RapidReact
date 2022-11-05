package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
	private final DriveSubsystem drive;
	private final MoInput input;

	public DriveCommand(DriveSubsystem drive, MoInput input) {
		this.drive = drive;
		this.input = input;

		addRequirements(drive);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.drive.arcadeDrive(input.getMoveRequest(), input.getTurnRequest());
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
