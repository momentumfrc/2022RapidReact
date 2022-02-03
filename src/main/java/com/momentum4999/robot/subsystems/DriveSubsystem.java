package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.RobotConfig;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.InputDevice.InputAxis;
import com.momentum4999.robot.util.Components;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	private final MotorController left = new VictorSP(Components.DR_LEFT);
	private final MotorController right = new VictorSP(Components.DR_RIGHT);
	private final Encoder driveEncoder = new Encoder(
		Components.DR_LEFT, Components.DR_RIGHT, true);

	private final MotorControllerGroup leftDriveGroup = new MotorControllerGroup(left);
	private final MotorControllerGroup rightDriveGroup = new MotorControllerGroup(right);

	private final DifferentialDrive driveTrain = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

	public DriveSubsystem() {
		this.driveTrain.setDeadband(0);

		this.driveEncoder.setDistancePerPulse(1.0 / RobotConfig.DRIVE_ENCODER_TPF);

		this.left.setInverted(true);

		// TODO: PID
	}

	public void drive(Mode mode, InputDevice input) {
		mode.drive(this.driveTrain, input);
	}

	public void stop() {
		this.driveTrain.stopMotor();
	}

	public enum Mode {
		TANK {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.tankDrive(
					input.getAxis(InputAxis.LY), input.getAxis(InputAxis.RY));
			}
		}, 
		ARCADE {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.arcadeDrive(
					input.getAxis(InputAxis.LY), input.getAxis(InputAxis.RX));
			}
		}, 
		CURVE {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.curvatureDrive(
					input.getAxis(InputAxis.LY), input.getAxis(InputAxis.RX), false);
			}
		};

		protected abstract void drive(DifferentialDrive driveTrain, InputDevice input);
	}
}
