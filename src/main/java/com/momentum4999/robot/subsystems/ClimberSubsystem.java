package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.Constants;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
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

	private boolean leftReady = false;
	private boolean rightReady = false;

	public ClimberSubsystem() {
	}

	public boolean leftRaiseLim() {
		return raiserLeft.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
	}

	public boolean rightRaiseLim() {
		return raiserRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
	}

	@Override
	public void periodic() {
		if (!leftReady) {
			raiserLeft.set(-0.4);

			if (this.leftRaiseLim()) {
				leftReady = true;
				raiserLeft.set(0);
				raiserLeft.getEncoder().setPosition(0);
			}
		}
		if (!rightReady) {
			raiserRight.set(-0.4);

			if (this.rightRaiseLim()) {
				rightReady = true;
				raiserRight.set(0);
				raiserRight.getEncoder().setPosition(0);
			}
		}

		SmartDashboard.putBoolean("Left Raise Limit", this.leftRaiseLim());
		SmartDashboard.putBoolean("Right Raise Limit", this.rightRaiseLim());

		SmartDashboard.putNumber("Left Raiser", this.raiserLeft.getEncoder().getPosition());
		SmartDashboard.putNumber("Right Raiser", this.raiserRight.getEncoder().getPosition());
		SmartDashboard.putNumber("Left Angle", this.adjustLeft.getEncoder().getPosition());
		SmartDashboard.putNumber("Right Angle", this.adjustRight.getEncoder().getPosition());
	}

	public void raise(double power) {
		power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();

		if (leftReady && rightReady) {
			if ((power > 0 && this.raiserLeft.getEncoder().getPosition() < Constants.CLIMBER_DISTANCE) ||
				(power < 0 && this.raiserLeft.getEncoder().getPosition() > 0 && !this.leftRaiseLim())) {
				this.raiserLeft.set(power);
			} else {
				this.raiserLeft.set(0);

				if (this.leftRaiseLim()) {
					this.raiserLeft.getEncoder().setPosition(0);
				}
			}

			if ((power > 0 && this.raiserRight.getEncoder().getPosition() < Constants.CLIMBER_DISTANCE) ||
				(power < 0 && this.raiserRight.getEncoder().getPosition() > 0 && !this.rightRaiseLim())) {
				this.raiserRight.set(power);
			} else {
				this.raiserRight.set(0);

				if (this.rightRaiseLim()) {
					this.raiserRight.getEncoder().setPosition(0);
				}
			}
		}

		/*
		if (power > 0) {
			if (this.raiserLeft.getEncoder().getPosition() < Constants.CLIMBER_DISTANCE) {
				this.raiserLeft.set(power);
			} else this.raiserLeft.set(0);
			
		} else {
			if (this.raiserLeft.getEncoder().getPosition() > 0) {
				this.raiserLeft.set(power);
			} else this.raiserLeft.set(0);
		}*/
	}

	public void idleRaisers() {
		this.raiserLeft.set(0);
		this.raiserRight.set(0);
	}

	public void adjust(double power) {
		power = power * MoPrefs.CLIMBER_ADJUST_SETPOINT.get();
		this.adjustLeft.set(power);
		this.adjustRight.set(power);
	}

	public void idleAdjustors() {
		this.adjustLeft.set(0);
		this.adjustRight.set(0);
	}

	public void stop() {
		this.idleRaisers();
		this.idleAdjustors();
	}

	private static boolean isWithinRange(CANSparkMax controller) {
		double pos = controller.getEncoder().getPosition();

		return pos > 0 && pos < Constants.CLIMBER_DISTANCE;
	}
}
