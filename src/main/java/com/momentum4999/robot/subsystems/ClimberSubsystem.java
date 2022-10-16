package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.Constants;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
	private final CANSparkMax raiserLeft = new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax raiserRight = new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax adjustLeft = new CANSparkMax(Components.CLIMB_ADJUST_L, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax adjustRight = new CANSparkMax(Components.CLIMB_ADJUST_R, CANSparkMaxLowLevel.MotorType.kBrushless);

	private CalibPhase leftReady = CalibPhase.READY;
	private CalibPhase rightReady = CalibPhase.READY;
	private CalibPhase lAngleReady = CalibPhase.READY;
	private CalibPhase rAngleReady = CalibPhase.READY;

	public ClimberSubsystem() {
	}

	public boolean leftRaiseLim() {
		return raiserLeft.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
	}

	public boolean rightRaiseLim() {
		return raiserRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
	}

	public boolean leftAngleLim() {
		return adjustLeft.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
	}

	public boolean rightAngleLim() {
		return adjustRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
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

		if (lAngleReady == CalibPhase.CALIBRATING) {
			adjustLeft.set(0.3);

			if (this.leftAngleLim()) {
				lAngleReady = CalibPhase.RESETTING;
				adjustLeft.set(0);
				adjustLeft.getEncoder().setPosition(MoPrefs.CLIMB_ADJUST_LIM.get());
			}
		} else if (lAngleReady == CalibPhase.RESETTING) {
			adjustLeft.set(-0.2);

			if (adjustLeft.getEncoder().getPosition() <= 0) {
				lAngleReady = CalibPhase.READY;
				adjustLeft.set(0);
			}
		}

		if (rAngleReady == CalibPhase.CALIBRATING) {
			adjustRight.set(0.3);

			if (this.rightAngleLim()) {
				rAngleReady = CalibPhase.RESETTING;
				adjustRight.set(0);
				adjustRight.getEncoder().setPosition(MoPrefs.CLIMB_ADJUST_LIM.get());
			}
		} else if (rAngleReady == CalibPhase.RESETTING) {
			adjustRight.set(-0.2);

			if (adjustRight.getEncoder().getPosition() <= 0) {
				rAngleReady = CalibPhase.READY;
				adjustRight.set(0);
			}
		}

		MoShuffleboard.putBoolean("Left Raise Limit", this.leftRaiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", this.rightRaiseLim());
		MoShuffleboard.putBoolean("Left Angle Limit", this.leftAngleLim());
		MoShuffleboard.putBoolean("Right Angle Limit", this.rightAngleLim());

		MoShuffleboard.putNumber("Left Raiser", this.raiserLeft.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", this.raiserRight.getEncoder().getPosition());
		MoShuffleboard.putNumber("Left Angle", this.adjustLeft.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Angle", this.adjustRight.getEncoder().getPosition());

		MoShuffleboard.putString("Left Adj State", this.lAngleReady.name());
		MoShuffleboard.putString("Right Adj State", this.rAngleReady.name());
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

	public void idleRaisers() {
		this.raiserLeft.set(0);
		this.raiserRight.set(0);
	}

	public void adjustLeft(double power) {
		power = power * MoPrefs.CLIMBER_ADJUST_SETPOINT.get();
		
		if (lAngleReady == CalibPhase.READY && rAngleReady == CalibPhase.READY) {
			if ((power > 0 && this.adjustLeft.getEncoder().getPosition() < MoPrefs.CLIMB_ADJUST_MAX.get()) ||
				(power < 0 && this.adjustLeft.getEncoder().getPosition() > MoPrefs.CLIMB_ADJUST_MIN.get())) {
				this.adjustLeft.set(power);
			} else {
				this.adjustLeft.set(0);
			}
		}
	}

	public void adjustRight(double power) {
		power = power * MoPrefs.CLIMBER_ADJUST_SETPOINT.get();
		
		if (lAngleReady == CalibPhase.READY && rAngleReady == CalibPhase.READY) {
			if ((power > 0 && this.adjustRight.getEncoder().getPosition() < MoPrefs.CLIMB_ADJUST_MAX.get()) ||
				(power < 0 && this.adjustRight.getEncoder().getPosition() > MoPrefs.CLIMB_ADJUST_MIN.get())) {
				this.adjustRight.set(power);
			} else {
				this.adjustRight.set(0);
			}
		}
	}

	public void idleAdjustors() {
		this.adjustLeft.set(0);
		this.adjustRight.set(0);
	}

	public void stop() {
		this.idleRaisers();
		this.idleAdjustors();
	}

	public enum CalibPhase {
		CALIBRATING, RESETTING, READY;
	}
}
