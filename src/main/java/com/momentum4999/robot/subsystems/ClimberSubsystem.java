package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.triggers.OvercurrentTrigger;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

	private static final int CLIMBER_MAX_RPM = 4500;

	private static final int CLIMBER_MIN_POS = 50;

	public static class ClimberSide {
		public final CANSparkMax raiser;
		public final SparkMaxPIDController raiserPid;
		public final OvercurrentTrigger currentTrigger;
		private boolean hasZero = false;

		public ClimberSide(CANSparkMax raiser, OvercurrentTrigger currentTrigger) {
			this.raiser = raiser;
			this.raiserPid = raiser.getPIDController();
			this.currentTrigger = currentTrigger;
		}

		public void invalidateZero() {
			hasZero = false;
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
				return currentTrigger.get();
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

			if(power < 0 && raiser.getEncoder().getPosition() <= CLIMBER_MIN_POS) {
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
		double currentLimit = MoPrefs.CLIMBER_LIMIT_CURRENT.get();
		double currentCutoffTime = MoPrefs.CLIMBER_LIMIT_TIME.get();

		var leftSparkMax = new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless);
		var rightSparkMax = new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftClimber = new ClimberSide(
			leftSparkMax,
			OvercurrentTrigger.makeForSparkMax(currentLimit, currentCutoffTime, leftSparkMax)
		);

		rightClimber = new ClimberSide(
			rightSparkMax,
			OvercurrentTrigger.makeForSparkMax(currentLimit, currentCutoffTime, rightSparkMax)
		);

		MoPrefs.CLIMBER_LIMIT_CURRENT.subscribe(current -> {
			leftClimber.currentTrigger.setCurrentLimit(current);
			rightClimber.currentTrigger.setCurrentLimit(current);
		});

		MoPrefs.CLIMBER_LIMIT_TIME.subscribe(time -> {
			leftClimber.currentTrigger.setCutoffTime(time);
			rightClimber.currentTrigger.setCutoffTime(time);
		});

		Shuffleboard.getTab("MoDashboard").add("Invalidate Climber Zeros", new InstantCommand(() -> {
			leftClimber.invalidateZero();
			rightClimber.invalidateZero();
		}));
	}

	@Override
	public void periodic() {
		MoShuffleboard.putBoolean("Left Raise Limit", leftClimber.raiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", rightClimber.raiseLim());

		MoShuffleboard.putBoolean("Left Has Zero", leftClimber.getHasZero());
		MoShuffleboard.putBoolean("Right Has Zero", rightClimber.getHasZero());

		MoShuffleboard.putNumber("Left Raiser", leftClimber.raiser.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", rightClimber.raiser.getEncoder().getPosition());

		SmartDashboard.putNumber("Left Raiser Current", this.leftClimber.raiser.getOutputCurrent());
		SmartDashboard.putNumber("Right Raiser Current", this.leftClimber.raiser.getOutputCurrent());
	}

	public void stop() {
		leftClimber.stop();
		rightClimber.stop();
	}
}
