package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.InputDevice.InputAxis;
import com.momentum4999.robot.subsystems.ClimberSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleOpCommand extends CommandBase {
	private final DriveSubsystem drive;
	private final ClimberSubsystem climb;
	private final InputDevice input;
	private final Mode driveMode;

	public TeleOpCommand(Mode driveMode, DriveSubsystem drive, ClimberSubsystem climb, InputDevice input) {
		this.drive = drive;
		this.climb = climb;
		this.input = input;
		this.driveMode = driveMode;

		addRequirements(drive, climb);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (this.input.getAxis(InputAxis.LT) > 0 || this.input.getAxis(InputAxis.RT) > 0) {
			this.climb.raise(-input.getAxis(InputAxis.LY));
			this.climb.adjust(-input.getAxis(InputAxis.RY));
		} else {
			this.drive.drive(this.driveMode, this.input);
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.stop();
		this.climb.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
