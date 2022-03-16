package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooter = new CANSparkMax(Components.SHOOTER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final RelativeEncoder shooterEncoder = shooter.getEncoder();
	private final SparkMaxPIDController shootPidController = shooter.getPIDController();
	private final DigitalInput fullSensor = new DigitalInput(Components.SHOOTER_FULL_SENSOR);

	private final TargetingSubsystem targeting;

	private State shooterState = State.DEFAULT;

	public ShooterSubsystem(TargetingSubsystem targeting) {
		this.targeting = targeting;

		this.indexer.setInverted(true);
		this.shootPidController.setP(0.00007, 0);
		this.shootPidController.setFF(0.00018, 0);
	}

	public boolean fullOfBalls() {
		return fullSensor.get();
	}

	public boolean shooterUpToSpeed() {
		return this.shooterEncoder.getVelocity() > MoPrefs.SHOOTER_SETPOINT.get() - MoPrefs.SHOOTER_TARGET_ERROR.get();
	}

	public void runIndexer(boolean rev) {
		this.indexer.set((rev ? -1 : 1) * MoPrefs.INDEXER_SETPOINT.get());
	}

	public void idleIndexer() {
		this.indexer.stopMotor();
	}

	public void runShooter() {
		this.shootPidController.setReference(MoPrefs.SHOOTER_SETPOINT.get(), CANSparkMax.ControlType.kVelocity, 0);
	}

	public void retractShooter() {
		this.shooter.set(-0.2);
	}

	public void runActive(boolean doTargeting) {
		if (doTargeting) {
			this.targeting.turnToTarget();
		}

		if (shooterState == State.DEFAULT) {
			this.shooterState = State.LOADING;
		} else if (shooterState == State.LOADING) {
			if (this.fullOfBalls()) {
				this.shooterState = State.RETRACTING;
			} else this.runIndexer(false);
		} else if (shooterState == State.RETRACTING) {
			if (this.fullOfBalls()) {
				this.retractShooter();
				this.runIndexer(true);
			} else {
				this.shooterState = State.SHOOTING;
			}
		} else if (shooterState == State.SHOOTING) {
			this.runShooter();

			if (this.shooterUpToSpeed() && (this.targeting.isAtTarget() || !doTargeting)) {
				this.runIndexer(false);
			} else this.idleIndexer();
		}
	}

	public void stop() {
		idle();
		this.shooterState = State.DEFAULT;
	}

	public void idleShooter() {
		this.shooter.stopMotor();
	}

	public void idle() {
		idleIndexer();
		idleShooter();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Shooter Flywheel Velocity", shooterEncoder.getVelocity());
	}

	public enum State {
		DEFAULT, // Idle
		LOADING, // Moves all balls up
		RETRACTING, // Once at limit, retract until no longer at limit
		SHOOTING; // Shoot

		public State next() {
			State[] values = values();
			return values[(this.ordinal() + 1) % values.length];
		}
	}
}
