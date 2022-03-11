package com.momentum4999.robot.commands;

import com.momentum4999.robot.RobotContainer;
import com.momentum4999.robot.util.MoCode;
import com.momentum4999.robot.util.MoCode.MoCodeRuntime;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCommand extends CommandBase {
	private final RobotContainer robot;
	private MoCodeRuntime runtime = null;

	public AutonomousCommand(RobotContainer robot) {
		this.robot = robot;
	}

	@Override
	public void execute() {
		if (this.runtime == null) {
			this.runtime = this.robot.autoScriptChooser.getSelected();
		}
		this.runtime.periodic();
	}

	@Override
	public void end(boolean interrupted) {
		this.runtime.robot.stopSubsystems();
	}
}
