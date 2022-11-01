package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoBaseInput;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleOpCommand extends CommandBase {
	private final DriveSubsystem drive;
	private final MoBaseInput input;
	private final Mode driveMode;

	public TeleOpCommand(Mode driveMode, DriveSubsystem drive, MoBaseInput input) {
		this.drive = drive;
		this.input = input;
		this.driveMode = driveMode;

		addRequirements(drive);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.drive.drive(this.driveMode, this.input);
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
