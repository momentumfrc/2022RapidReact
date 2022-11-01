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

	public static class ClimberSide {
		public final CANSparkMax raiser;
		public final OvercurrentTrigger currentTrigger;
		private boolean hasZero = false;

		public ClimberSide(CANSparkMax raiser, OvercurrentTrigger currentTrigger) {
			this.raiser = raiser;
			this.currentTrigger = currentTrigger;
		}

		public boolean getHasZero() {
			return hasZero;
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
				raiser.set(power);

				if(raiseLim()) {
					raiser.getEncoder().setPosition(0);
					hasZero = true;
				}

				return;
			}

			if(power > 0 && raiser.getEncoder().getPosition() >= MoPrefs.CLIMB_HEIGHT.get()) {
				raiser.set(0);
				return;
			}

			if(power < 0 && raiseLim()) {
				raiser.getEncoder().setPosition(0);
				raiser.set(0);
				return;
			}


			if(power < 0 && raiser.getEncoder().getPosition() < 0) {
				raiser.set(0);
				return;
			}

			raiser.set(power);
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
			raiser.set(0);
		}
	}

	public final ClimberSide leftClimber;
	public final ClimberSide rightClimber;

	private final PowerDistribution pdp;

	public ClimberSubsystem(PowerDistribution pdp) {
		double currentLimit = MoPrefs.CLIMBER_LIMIT_CURRENT.get();
		double currentCutoffTime = MoPrefs.CLIMBER_LIMIT_TIME.get();

		leftClimber = new ClimberSide(
			new CANSparkMax(Components.CLIMB_RAISE_L, CANSparkMaxLowLevel.MotorType.kBrushless),
			OvercurrentTrigger.makeForPdp(currentLimit, currentCutoffTime, pdp, Components.CLIMB_RAISE_L_PDP)
		);

		rightClimber = new ClimberSide(
			new CANSparkMax(Components.CLIMB_RAISE_R, CANSparkMaxLowLevel.MotorType.kBrushless),
			OvercurrentTrigger.makeForPdp(currentLimit, currentCutoffTime, pdp, Components.CLIMB_RAISE_R_PDP)
		);

		this.pdp = pdp;

		MoPrefs.CLIMBER_LIMIT_CURRENT.subscribe(current -> {
			leftClimber.currentTrigger.setCurrentLimit(current);
			rightClimber.currentTrigger.setCurrentLimit(current);
		});

		MoPrefs.CLIMBER_LIMIT_TIME.subscribe(time -> {
			leftClimber.currentTrigger.setCutoffTime(time);
			rightClimber.currentTrigger.setCutoffTime(time);
		});
	}

	@Override
	public void periodic() {
		MoShuffleboard.putBoolean("Left Raise Limit", leftClimber.raiseLim());
		MoShuffleboard.putBoolean("Right Raise Limit", rightClimber.raiseLim());

		MoShuffleboard.putBoolean("Left Has Zero", leftClimber.getHasZero());
		MoShuffleboard.putBoolean("Right Has Zero", rightClimber.getHasZero());

		MoShuffleboard.putNumber("Left Raiser", leftClimber.raiser.getEncoder().getPosition());
		MoShuffleboard.putNumber("Right Raiser", rightClimber.raiser.getEncoder().getPosition());

		MoShuffleboard.putNumber("Left Raiser Current", this.pdp.getCurrent(Components.CLIMB_RAISE_L_PDP));
	}

	public void stop() {
		leftClimber.stop();
		rightClimber.stop();
	}
}
