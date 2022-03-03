package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoUtil;
import com.momentum4999.robot.util.MotorRamp;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	public final MotorController intakeRoller = new VictorSP(Components.INTAKE_ROLLER);
	private final DoubleSolenoid intakePiston = new DoubleSolenoid(
		PneumaticsModuleType.CTREPCM, Components.INTAKE_PISTON_EXTEND, Components.INTAKE_PISTON_RETRACT);
	private final MotorRamp motorRamp = new MotorRamp(0.1);

	public IntakeSubsystem() {
		this.intakePiston.set(Value.kReverse);
	}

	public void runIntake(boolean rev) {
		this.updateMotorsRamped(MoPrefs.INTAKE_ROLLER_SETPOINT.get() * (rev ? -1 : 1));
	}

	public void idleIntake() {
		this.updateMotorsRamped(0);
	}

	public void updateMotorsRamped(double pwr) {
		this.motorRamp.run(pwr);
		this.updateMotors(this.motorRamp.power());
	}

	public void updateMotors(double pwr) {
		this.intakeRoller.set(pwr);
	}

	public void toggleIntakeExtension() {
		this.intakePiston.set(MoUtil.getSolenoidOpposite(this.intakePiston.get()));
	}

	public void setIntake(boolean extended) {
		this.intakePiston.set(extended ? Value.kForward : Value.kReverse);
	}
}
