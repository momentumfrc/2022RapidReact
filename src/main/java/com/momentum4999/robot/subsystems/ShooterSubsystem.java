package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooter = new CANSparkMax(Components.SHOOTER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final RelativeEncoder shooterEncoder = shooter.getEncoder();
	private final DigitalInput fullSensor = new DigitalInput(Components.SHOOTER_FULL_SENSOR);

	public ShooterSubsystem() {
		this.indexer.setInverted(true);
	}

	public boolean fullOfBalls() {
		return fullSensor.get();
	}

	public boolean shooterUpToSpeed() {
		return Math.abs(shooterEncoder.getVelocity() - MoPrefs.SHOOTER_TARGET.get()) < MoPrefs.SHOOTER_TARGET_ERROR.get();
	}

	public void runIndexer(boolean rev) {
		this.indexer.set((rev ? -1 : 1) * MoPrefs.INDEXER_SETPOINT.get());
	}

	public void idleIndexer() {
		this.shooter.set(0);
	}

	public void runShooter() {
		this.shooter.set(MoPrefs.SHOOTER_SETPOINT.get());
	}

	public void idleShooter() {
		this.shooter.set(0);
	}

	public void idle() {
		idleIndexer();
		idleShooter();
	}
}
