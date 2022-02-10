package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.InputDevice.InputAxis;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveCommand extends CommandBase {
	private final DriveSubsystem subsystem;
	private final InputDevice input;
	private final Mode driveMode;

	public DriveCommand(Mode driveMode, DriveSubsystem subsystem, InputDevice input) {
		this.subsystem = subsystem;
		this.input = input;
		this.driveMode = driveMode;

		addRequirements(subsystem);
	}

	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		subsystem.drive(this.driveMode, this.input);
	}

	@Override
	public void end(boolean interrupted) {
		subsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
