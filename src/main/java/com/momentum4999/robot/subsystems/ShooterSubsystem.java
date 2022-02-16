package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooter = new CANSparkMax(Components.SHOOTER, CANSparkMaxLowLevel.MotorType.kBrushless);

	public ShooterSubsystem() {}

	private boolean full() {
		// TODO: Use sensor to detect when filled with 2 balls

		return false;
	}

	public void indexWhileIntaking(boolean rev) {
		if (!full()) {
			runIndexer(rev ? -1 : 1);
		} else {
			idleIndexer();
		}
	}

	public void shootAndIndex() {
		if (!full()) {
			runIndexer(1);
		} else {
			idleIndexer();
		}

		runShooter(1);
	}

	public void runIndexer(double pwr) {
		this.indexer.set(pwr);
	}

	public void idleIndexer() {
		runIndexer(0);
	}

	public void runShooter(double pwr) {
		this.shooter.set(pwr);
	}

	public void idleShooter() {
		runShooter(0);
	}
}
