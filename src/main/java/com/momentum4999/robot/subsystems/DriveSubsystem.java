package com.momentum4999.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.momentum4999.robot.RobotConfig;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.InputDevice.InputAxis;
import com.momentum4999.robot.util.Components;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	private final MotorController leftFront = new WPI_TalonFX(Components.DR_LEFT_FRONT);
	private final MotorController leftRear = new WPI_TalonFX(Components.DR_LEFT_REAR);
	private final MotorController rightFront = new WPI_TalonFX(Components.DR_RIGHT_FRONT);
	private final MotorController rightRear = new WPI_TalonFX(Components.DR_RIGHT_REAR);

	private final Encoder leftDriveEncoder = new Encoder(
		Components.DR_LEFT_FRONT, Components.DR_LEFT_REAR, true);
	private final Encoder rightDriveEncoder = new Encoder(
		Components.DR_RIGHT_FRONT, Components.DR_RIGHT_REAR, false);

	private final MotorControllerGroup leftDriveGroup = new MotorControllerGroup(leftFront, leftRear);
	private final MotorControllerGroup rightDriveGroup = new MotorControllerGroup(rightFront, rightRear);

	private final DifferentialDrive driveTrain = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

	public DriveSubsystem() {
		this.driveTrain.setDeadband(0);

		this.leftDriveEncoder.setDistancePerPulse(1.0 / RobotConfig.DRIVE_ENCODER_TPF);
    	this.rightDriveEncoder.setDistancePerPulse(1.0 / RobotConfig.DRIVE_ENCODER_TPF);

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
