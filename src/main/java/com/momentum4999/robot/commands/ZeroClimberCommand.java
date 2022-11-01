package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.ClimberSubsystem;
import com.momentum4999.robot.triggers.OvercurrentTrigger;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroClimberCommand extends CommandBase {
	private final static double ZEROING_POWER = 0.05;

	private final OvercurrentTrigger raiserLeftCurrentTrigger;
	private final OvercurrentTrigger raiserRightCurrentTrigger;

	private final ClimberSubsystem climber;

	public ZeroClimberCommand(ClimberSubsystem climber, PowerDistribution pdp) {
		this.climber = climber;
		addRequirements(climber);

		double currentLimit = MoPrefs.CLIMBER_LIMIT_CURRENT.get();
		double currentCutoffTime = MoPrefs.CLIMBER_LIMIT_TIME.get();

		raiserLeftCurrentTrigger = OvercurrentTrigger.makeForPdp(currentLimit, currentCutoffTime, pdp, Components.CLIMB_RAISE_L_PDP);
		raiserRightCurrentTrigger = OvercurrentTrigger.makeForPdp(currentLimit, currentCutoffTime, pdp, Components.CLIMB_RAISE_R_PDP);

		MoPrefs.CLIMBER_LIMIT_CURRENT.subscribe(current -> {
			raiserLeftCurrentTrigger.setCurrentLimit(current);
			raiserRightCurrentTrigger.setCurrentLimit(current);
		});

		MoPrefs.CLIMBER_LIMIT_TIME.subscribe(time -> {
			raiserLeftCurrentTrigger.setCutoffTime(time);
			raiserRightCurrentTrigger.setCutoffTime(time);
		});
	}

	@Override
	public void execute() {
		if (climber.leftReady == ClimberSubsystem.CalibPhase.CALIBRATING) {
			climber.raiserLeft.set(-ZEROING_POWER);

			if (climber.leftRaiseLim()) {
				climber.leftReady = ClimberSubsystem.CalibPhase.RESETTING;
				climber.raiserLeft.set(0);
			}
		} else if (climber.leftReady == ClimberSubsystem.CalibPhase.RESETTING) {
			climber.raiserLeft.set(ZEROING_POWER);

			if (!climber.leftRaiseLim()) {
				climber.leftReady = ClimberSubsystem.CalibPhase.READY;
				climber.raiserLeft.set(0);
				climber.raiserLeft.getEncoder().setPosition(0);
			}
		}
		climber.raiserRight.set(0);
		/*if (climber.rightReady == ClimberSubsystem.CalibPhase.CALIBRATING) {
			climber.raiserRight.set(-ZEROING_POWER);

			if (climber.rightRaiseLim()) {
				climber.rightReady = ClimberSubsystem.CalibPhase.RESETTING;
				climber.raiserRight.set(0);
			}
		} else if (climber.rightReady == ClimberSubsystem.CalibPhase.RESETTING) {
			climber.raiserRight.set(ZEROING_POWER);

			if (!climber.rightRaiseLim()) {
				climber.rightReady = ClimberSubsystem.CalibPhase.READY;
				climber.raiserRight.set(0);
				climber.raiserRight.getEncoder().setPosition(0);
			}
		}*/
	}

	@Override
	public boolean isFinished() {
		return climber.leftReady == ClimberSubsystem.CalibPhase.READY && climber.rightReady == ClimberSubsystem.CalibPhase.READY;
	}
}
