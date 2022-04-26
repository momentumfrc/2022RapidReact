package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooterA = new CANSparkMax(Components.SHOOTER_A, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooterB = new CANSparkMax(Components.SHOOTER_B, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax hood = new CANSparkMax(Components.HOOD, CANSparkMaxLowLevel.MotorType.kBrushless);

	private final RelativeEncoder shooterAEncoder = shooterA.getEncoder();
	private final RelativeEncoder shooterBEncoder = shooterB.getEncoder();
	private final SparkMaxPIDController shootAPidController = shooterA.getPIDController();
	private final SparkMaxPIDController shootBPidController = shooterB.getPIDController();

	private final DigitalInput fullSensor = new DigitalInput(Components.SHOOTER_FULL_SENSOR);

	private final TargetingSubsystem targeting;

	private State shooterState = State.DEFAULT;

	public ShooterSubsystem(TargetingSubsystem targeting) {
		this.targeting = targeting;

		this.indexer.setInverted(true);
		this.shooterB.setInverted(true);
		this.shooterB.follow(shooterA, true);
		this.hood.setInverted(true);

		this.shootAPidController.setP(0.00008, 0);
		this.shootAPidController.setFF(0.00018, 0);
		this.shootBPidController.setP(0.00007, 0);
		this.shootBPidController.setFF(0.00018, 0);
	}

	public boolean fullOfBalls() {
		return fullSensor.get();
	}

	public boolean shooterUpToSpeed() {
		double min = MoPrefs.SHOOTER_SETPOINT.get() - MoPrefs.SHOOTER_TARGET_ERROR.get();
		return this.shooterAEncoder.getVelocity() > min;
	}

	public boolean hoodFullyOpen(boolean calc) {
		return this.hood.getEncoder().getPosition() >= this.calcHoodTarget(calc);
	}

	public void runIndexer(boolean rev) {
		this.indexer.set((rev ? -0.5 : 1) * MoPrefs.INDEXER_SETPOINT.get());
	}

	public void idleIndexer() {
		this.indexer.stopMotor();
	}

	public void runShooter() {
		this.shootAPidController.setReference(MoPrefs.SHOOTER_SETPOINT.get(), CANSparkMax.ControlType.kVelocity, 0);
	}

	public void retractShooter() {
		this.shooterA.set(-0.2);
	}

	private double calcHoodTarget(boolean calc) {
		return MoPrefs.HOOD_DISTANCE_TEST.get();

		//double x = this.targeting.getTargetDistance();
		//return Utils.clip(calc ? -1.5 * Math.pow(x - 6, 2) + 37 : 2, 0, 50);
	}

	public void openHood(boolean calc) {
		if (!this.hoodFullyOpen(calc)) {
			//double range = this.calcHoodTarget(calc);
			//double offset = Utils.clip(range - this.hood.getEncoder().getPosition(), 0, range);
			//this.hood.set(Utils.clip(MoUtil.approachedPowerCalc(offset, range, 0.2, 1) * MoPrefs.HOOD_SETPOINT.get(), 0, 1));
			this.hood.set(0.4);
		} else {
			this.hood.set(0);
		}
	}

	public void closeHood() {
		if (this.hood.getEncoder().getPosition() > 0.7) {
			//this.hood.set(-0.2 * MoPrefs.HOOD_SETPOINT.get());
			this.hood.set(-0.2);
		} else {
			this.hood.set(0);
		}
	}

	public void runActive(boolean doTargeting) {
		if (doTargeting) {
			this.targeting.turnToTarget();
		}

		this.openHood(doTargeting);

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
			} else if (this.hoodFullyOpen(doTargeting)) {
				this.shooterState = State.SHOOTING;
			} else {
				this.idleIndexer();
			}
		} else if (shooterState == State.SHOOTING) {
			this.runShooter();

			if (this.shooterUpToSpeed() && (this.targeting.isAtTarget() || !doTargeting)) {
				this.runIndexer(false);
			} else this.idleIndexer();

			if (!doTargeting) {
				this.targeting.resetTargetPose();
			}
		}
	}

	public void stop() {
		idle();
		this.shooterState = State.DEFAULT;
	}

	public void idleShooter() {
		this.shooterA.stopMotor();
		this.shooterB.stopMotor();
	}

	public void idle() {
		idleIndexer();
		idleShooter();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Flywheel Motor A Velocity", shooterAEncoder.getVelocity());
		SmartDashboard.putNumber("Flywheel Motor B Velocity", shooterBEncoder.getVelocity());
		SmartDashboard.putNumber("Current Hood Dist", this.hood.getEncoder().getPosition());
		SmartDashboard.putString("Shooter State", this.shooterState.name());
		SmartDashboard.putBoolean("Full of Ball", this.fullOfBalls());
	
		if (this.shooterState == State.DEFAULT) {
			this.closeHood();
		}

		this.shootAPidController.setP(MoPrefs.SHOOTER_KP.get());
		this.shootAPidController.setFF(MoPrefs.SHOOTER_KFF.get());
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
