package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoDriveCommand extends Command {
	private final DriveSubsystem drive;
	private final double power_left;
	private final double power_right;
	private final double distance;

	private double startDistance;

	public AutoDriveCommand(DriveSubsystem drive, double power_left, double power_right, double distance) {
		this.drive = drive;
		this.power_left = power_left;
		this.power_right = power_right;
		this.distance = distance;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		this.startDistance = drive.getAverageDrivenDistance();
	}

	@Override
	public void execute() {
		drive.driveDirect(power_left, power_right);
	}

	@Override
	public void end(boolean interrupted) {
		drive.stop();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(drive.getAverageDrivenDistance() - this.startDistance) > distance;
	}
}
