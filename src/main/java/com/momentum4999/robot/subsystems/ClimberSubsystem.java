package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.triggers.OvercurrentTrigger;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private final CANSparkMax raiserLeft = new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax raiserRight = new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final OvercurrentTrigger raiserLeftCurrentTrigger;
	private final OvercurrentTrigger raiserRightCurrentTrigger;

	private CalibPhase leftReady = CalibPhase.READY;
	private CalibPhase rightReady = CalibPhase.READY;

	public ClimberSubsystem(PowerDistribution pdp) {
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

	public boolean leftRaiseLim() {
		if(MoPrefs.CLIMBER_USE_LIMIT_SWITCHES.get()) {
			return raiserLeft.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
		} else {
			return raiserLeftCurrentTrigger.get();
		}
	}

	public boolean rightRaiseLim() {
		if(MoPrefs.CLIMBER_USE_LIMIT_SWITCHES.get()) {
			return raiserRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
		} else {
			return raiserRightCurrentTrigger.get();
		}
	}

	@Override
	public void periodic() {
		if (leftReady == CalibPhase.CALIBRATING) {
			raiserLeft.set(-0.3);

			if (this.leftRaiseLim()) {
				leftReady = CalibPhase.RESETTING;
				raiserLeft.set(0);
			}
		} else if (leftReady == CalibPhase.RESETTING) {
			raiserLeft.set(0.1);

			if (!this.leftRaiseLim()) {
				leftReady = CalibPhase.READY;
				raiserLeft.set(0);
				raiserLeft.getEncoder().setPosition(0);
			}
		}
		if (rightReady == CalibPhase.CALIBRATING) {
			raiserRight.set(-0.3);

			if (this.rightRaiseLim()) {
				rightReady = CalibPhase.RESETTING;
				raiserRight.set(0);
			}
		} else if (rightReady == CalibPhase.RESETTING) {
			raiserRight.set(0.1);

			if (!this.rightRaiseLim()) {
				rightReady = CalibPhase.READY;
				raiserRight.set(0);
				raiserRight.getEncoder().setPosition(0);
			}
		}

		MoShuffleboard.putBoolean("Left Raise Limit", this.leftRaiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", this.rightRaiseLim());

		MoShuffleboard.putNumber("Left Raiser", this.raiserLeft.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", this.raiserRight.getEncoder().getPosition());
	}

	public void raiseLeft(double power) {
		power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();

		if (leftReady == CalibPhase.READY && rightReady == CalibPhase.READY) {
			if ((power > 0 && this.raiserLeft.getEncoder().getPosition() < MoPrefs.CLIMB_HEIGHT.get()) ||
				(power < 0 && this.raiserLeft.getEncoder().getPosition() > 0 && !this.leftRaiseLim())) {
				this.raiserLeft.set(power);
			} else {
				this.raiserLeft.set(0);

				if (this.leftRaiseLim()) {
					this.raiserLeft.getEncoder().setPosition(0);
				}
			}
		}
	}

	public void raiseRight(double power) {
		power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();
		this.raiserRight.set(power);

		if (leftReady == CalibPhase.READY && rightReady == CalibPhase.READY) {
			if ((power > 0 && this.raiserRight.getEncoder().getPosition() < MoPrefs.CLIMB_HEIGHT.get()) ||
				(power < 0 && this.raiserRight.getEncoder().getPosition() > 0 && !this.rightRaiseLim())) {
				this.raiserRight.set(power);
			} else {
				this.raiserRight.set(0);

				if (this.rightRaiseLim()) {
					this.raiserRight.getEncoder().setPosition(0);
				}
			}
		}
	}

	public void stop() {
		this.raiserLeft.set(0);
		this.raiserRight.set(0);
	}

	public enum CalibPhase {
		CALIBRATING, RESETTING, READY;
	}
}
