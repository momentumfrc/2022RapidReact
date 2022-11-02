package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.triggers.StallTrigger;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimberSubsystem extends SubsystemBase {

	private static final int CLIMBER_MAX_RPM = 4500;

	public static class ClimberSide {
		public final CANSparkMax raiser;
		public final SparkMaxPIDController raiserPid;
		public final Trigger trigger;
		private boolean hasZero = false;

		public ClimberSide(CANSparkMax raiser, Trigger trigger) {
			this.raiser = raiser;
			this.raiserPid = raiser.getPIDController();
			this.trigger = trigger;
		}

		public boolean getHasZero() {
			return hasZero;
		}

		private void set(double power) {
			if(MoPrefs.CLIMBER_USE_PID.get()) {
				raiserPid.setReference(power * CLIMBER_MAX_RPM, CANSparkMax.ControlType.kVelocity);
			} else {
				raiserPid.setReference(power, CANSparkMax.ControlType.kDutyCycle);
			}
		}

		public boolean raiseLim() {
			if(MoPrefs.CLIMBER_USE_LIMIT_SWITCHES.get()) {
				return raiser.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
			} else {
				return trigger.get();
			}
		}

		public void raise(double power) {
			power *= MoPrefs.CLIMBER_RAISE_SETPOINT.get();

			if(!hasZero) {
				// If we don't have a reliable zero, ignore any logic based on encoders
				set(power);

				return;
			}

			if(power > 0 && raiser.getEncoder().getPosition() >= MoPrefs.CLIMB_HEIGHT.get()) {
				set(0);
				return;
			}

			if(power < 0 && raiser.getEncoder().getPosition() < 0) {
				set(0);
				return;
			}

			set(power);
		}

		public void zero(double power) {
			if (!hasZero) {
				raiser.set(-power);

				if (raiseLim()) {
					raiser.getEncoder().setPosition(0);
					hasZero = true;
					raiser.set(0);
				}
			} else {
				raiser.set(0);
			}
		}

		public void stop() {
			raiser.stopMotor();
		}
	}

	public final ClimberSide leftClimber;
	public final ClimberSide rightClimber;

	public ClimberSubsystem() {
		var leftClimberMax = new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
		var rightClimberMax = new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftClimber = new ClimberSide(
			leftClimberMax,
			StallTrigger.makeForSparkMax(leftClimberMax)
		);

		rightClimber = new ClimberSide(
			rightClimberMax,
			StallTrigger.makeForSparkMax(rightClimberMax)
		);
	}

	@Override
	public void periodic() {
		MoShuffleboard.putBoolean("Left Raise Limit", leftClimber.raiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", rightClimber.raiseLim());

		MoShuffleboard.putBoolean("Left Has Zero", leftClimber.getHasZero());
		MoShuffleboard.putBoolean("Right Has Zero", rightClimber.getHasZero());

		MoShuffleboard.putNumber("Left Raiser", leftClimber.raiser.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", rightClimber.raiser.getEncoder().getPosition());
	}

	public void stop() {
		leftClimber.stop();
		rightClimber.stop();
	}
}
