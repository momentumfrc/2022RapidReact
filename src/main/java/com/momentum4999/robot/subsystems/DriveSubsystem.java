package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.RobotConfig;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoShuffleboard;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	private final MotorController left = new VictorSP(Components.DR_LEFT);
	private final MotorController right = new VictorSP(Components.DR_RIGHT);
	private final Encoder leftEncoder = new Encoder(
		Components.ENC_LEFT_A, Components.ENC_LEFT_B, true);
	private final Encoder rightEncoder = new Encoder(
		Components.ENC_RIGHT_A, Components.ENC_RIGHT_B, true);

	private final DifferentialDrive driveTrain = new DifferentialDrive(left, right);

	public DriveSubsystem() {
		this.driveTrain.setDeadband(0);

		this.leftEncoder.setDistancePerPulse(-1f / RobotConfig.DRIVE_ENCODER_TPF);
		this.rightEncoder.setDistancePerPulse(1f / RobotConfig.DRIVE_ENCODER_TPF);

		this.right.setInverted(true);

		// TODO: PID
	}

	public void arcadeDrive(double move_request, double turn_request) {
		this.driveTrain.arcadeDrive(move_request, turn_request);
	}

	public void driveDirect(double left, double right) {
		this.driveTrain.tankDrive(left, right);
	}

	public void stop() {
		this.driveTrain.stopMotor();
	}

	@Override
	public void periodic() {
		MoShuffleboard.putNumber("Average Distance", this.getAverageDrivenDistance());
		MoShuffleboard.putNumber("Left Distance", this.leftEncoder.getDistance());
		MoShuffleboard.putNumber("Right Distance", this.rightEncoder.getDistance());
	}

	public double getAverageDrivenDistance() {
		return (this.leftEncoder.getDistance() + this.rightEncoder.getDistance()) * 0.5;
	}
}
