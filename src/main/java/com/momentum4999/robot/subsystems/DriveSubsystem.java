package com.momentum4999.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.momentum4999.robot.RobotConfig;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.InputDevice.InputAxis;
import com.momentum4999.robot.util.Components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	private final MotorController left = new VictorSP(Components.DR_LEFT);
	private final MotorController right = new VictorSP(Components.DR_RIGHT);
	private final Encoder leftEncoder = new Encoder(
		Components.ENC_LEFT_A, Components.ENC_LEFT_B, true);
	private final Encoder rightEncoder = new Encoder(
		Components.ENC_RIGHT_A, Components.ENC_RIGHT_B, true);

	private final Gyro gyro = new AHRS(SerialPort.Port.kMXP);

	private final MotorControllerGroup leftDriveGroup = new MotorControllerGroup(left);
	private final MotorControllerGroup rightDriveGroup = new MotorControllerGroup(right);

	private final DifferentialDrive driveTrain = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
	private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

	public DriveSubsystem() {
		this.driveTrain.setDeadband(0);

		this.leftEncoder.setDistancePerPulse(-1f / RobotConfig.DRIVE_ENCODER_TPF);
		this.rightEncoder.setDistancePerPulse(1f / RobotConfig.DRIVE_ENCODER_TPF);

		this.right.setInverted(true);

		// TODO: PID
	}

	public void drive(Mode mode, InputDevice input) {
		mode.drive(this.driveTrain, input);
	}

	public void driveDirect(double left, double right) {
		this.driveTrain.tankDrive(left, right);
	}

	public void stop() {
		this.driveTrain.stopMotor();
	}

	@Override
	public void periodic() {
		this.odometry.update(this.gyro.getRotation2d(), this.leftEncoder.getDistance(), this.rightEncoder.getDistance());
		
		SmartDashboard.putNumber("Average Distance", this.getAverageDrivenDistance());
		SmartDashboard.putNumber("Left Distance", this.leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Distance", this.rightEncoder.getDistance());
		SmartDashboard.putNumber("Gyro Degrease", this.getPose().getRotation().getDegrees());
	}

	public double getAverageDrivenDistance() {
		return (this.leftEncoder.getDistance() + this.rightEncoder.getDistance()) * 0.5;
	}

	public Pose2d getPose() {
		return this.odometry.getPoseMeters();
	}

	public enum Mode {
		TANK {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.tankDrive(
					-input.getAxis(InputAxis.LY), -input.getAxis(InputAxis.RY));
			}
		}, 
		ARCADE {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.arcadeDrive(
					-input.getAxis(InputAxis.LY), input.getAxis(InputAxis.RX));
			}
		}, 
		CURVE {
			@Override protected void drive(DifferentialDrive driveTrain, InputDevice input) {
				driveTrain.curvatureDrive(
					-input.getAxis(InputAxis.LY), input.getAxis(InputAxis.RX), false);
			}
		};

		protected abstract void drive(DifferentialDrive driveTrain, InputDevice input);
	}
}
