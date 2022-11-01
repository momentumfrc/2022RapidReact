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
	public final CANSparkMax raiserLeft = new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
	public final CANSparkMax raiserRight = new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final OvercurrentTrigger raiserLeftCurrentTrigger;
	private final OvercurrentTrigger raiserRightCurrentTrigger;

	public CalibPhase leftReady = CalibPhase.CALIBRATING;
	public CalibPhase rightReady = CalibPhase.CALIBRATING;

	private final PowerDistribution pdp;

	public ClimberSubsystem(PowerDistribution pdp) {
		double currentLimit = MoPrefs.CLIMBER_LIMIT_CURRENT.get();
		double currentCutoffTime = MoPrefs.CLIMBER_LIMIT_TIME.get();

		this.pdp = pdp;

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
		MoShuffleboard.putBoolean("Left Raise Limit", this.leftRaiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", this.rightRaiseLim());

		MoShuffleboard.putString("Left CalibPhase", leftReady.toString());
		MoShuffleboard.putString("Right CalibPhase", rightReady.toString());

		MoShuffleboard.putNumber("Left Raiser", this.raiserLeft.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", this.raiserRight.getEncoder().getPosition());

		MoShuffleboard.putNumber("Left Raiser Current", this.pdp.getCurrent(Components.CLIMB_RAISE_L_PDP));
	}

	public void raiseLeft(double power) {
		power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();

		if(leftReady != CalibPhase.READY) {
			// If we don't have a reliable zero, ignore any logic based on encoders
			this.raiserLeft.set(power);

			if(leftRaiseLim()) {
				this.raiserLeft.getEncoder().setPosition(0);
				leftReady = CalibPhase.READY;
			}

			return;
		}

		if(power > 0 && this.raiserLeft.getEncoder().getPosition() >= MoPrefs.CLIMB_HEIGHT.get()) {
			this.raiserLeft.set(0);
			return;
		}

		if(power < 0 && leftRaiseLim()) {
			this.raiserLeft.getEncoder().setPosition(0);
			this.raiserLeft.set(0);
			return;
		}


		if(power < 0 && this.raiserLeft.getEncoder().getPosition() < 0) {
			this.raiserLeft.set(0);
			return;
		}

		this.raiserLeft.set(power);
	}

	public void raiseRight(double power) {
		power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();

		if(rightReady != CalibPhase.READY) {
			// If we don't have a reliable zero, ignore any logic based on encoders
			this.raiserRight.set(power);

			if(rightRaiseLim()) {
				this.raiserRight.getEncoder().setPosition(0);
				rightReady = CalibPhase.READY;
			}

			return;
		}

		if(power > 0 && this.raiserRight.getEncoder().getPosition() >= MoPrefs.CLIMB_HEIGHT.get()) {
			this.raiserRight.set(0);
			return;
		}

		if(power < 0 && rightRaiseLim()) {
			this.raiserRight.getEncoder().setPosition(0);
			this.raiserRight.set(0);
			return;
		}


		if(power < 0 && this.raiserRight.getEncoder().getPosition() < 0) {
			this.raiserRight.set(0);
			return;
		}

		this.raiserRight.set(power);
	}

	public void stop() {
		this.raiserLeft.set(0);
		this.raiserRight.set(0);
	}

	public enum CalibPhase {
		CALIBRATING, RESETTING, READY;
	}
}
