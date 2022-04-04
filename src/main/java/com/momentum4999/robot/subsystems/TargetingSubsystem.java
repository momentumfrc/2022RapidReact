package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.Constants;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetingSubsystem extends SubsystemBase {
	private final DriveSubsystem driveSubsystem;
	public final LimelightTableAdapter limelight = new LimelightTableAdapter();

	private Pose2d targetPose;
	private boolean hasFirstInit;

	private Translation2d lastLLPos = null;
	private int llConfidence = 0;

	public TargetingSubsystem(DriveSubsystem drive) {
		this.driveSubsystem = drive;

		resetTargetPose();
		hasFirstInit = false;
	}

	public void resetTargetPose() {
		Pose2d pose = this.driveSubsystem.getPose();
		double yaw = pose.getRotation().getDegrees();
		Translation2d offset = new Translation2d(
			Math.cos(Math.toRadians(yaw)),
			Math.sin(Math.toRadians(yaw))).times(Constants.ROBOT_START_DISTANCE_FROM_GOAL_M);
		this.targetPose = pose.plus(new Transform2d(offset, new Rotation2d(0)).inverse());
		
		SmartDashboard.putString("Target Position", this.targetPose.getTranslation().toString());
		this.hasFirstInit = true;
	}

	public void turnToTarget() {
		double power = MoUtil.approachedPowerCalc(
			this.getTargetAngleDifference(), 180, 
			MoPrefs.TARGETING_MIN_PWR.get(), 
			MoPrefs.TARGETING_FALLOFF.get());

		if (!this.isAtTarget()) {
			this.driveSubsystem.driveDirect(power, -power);
		}
	}

	public boolean isAtTarget() {
		return Math.abs(this.getTargetAngleDifference()) < MoPrefs.TARGETING_ANGLE_ERROR.get();
	}

	public double getTargetAngleDifference() {
		Pose2d pose = getPoseRelativeToTarget();
		double angle = MoUtil.wrapAngleDeg(-Math.toDegrees(Math.atan2(-pose.getTranslation().getY(), pose.getTranslation().getX())));
		double robotAngle = MoUtil.wrapAngleDeg(pose.getRotation().getDegrees());
		
		return MoUtil.wrapAngleDeg(robotAngle - angle);
	}

	public double getTargetDistance() {
		Pose2d pose = getPoseRelativeToTarget();

		return Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2));
	}

	public boolean hasFirstInit() {
		return this.hasFirstInit;
	}

	@Override
	public void periodic() {
		this.limelight.periodic();

		SmartDashboard.putString("Robot Position", this.driveSubsystem.getPose().toString());
		SmartDashboard.putNumber("Angle to Target", this.getTargetAngleDifference());
		SmartDashboard.putNumber("Distance to Target", this.getTargetDistance());

		SmartDashboard.putString("Target Offset Pose", this.getPoseRelativeToTarget().toString());

		this.limelight.ifTarget(this::setTargetFromLimelight);
	}

	private void setTargetFromLimelight(double hAngle, double vAngle, double area) {
		Pose2d pose = this.driveSubsystem.getPose();
		double opposite = Constants.LL_GOAL_HEIGHT_M - Constants.LL_HEIGHT_M;
		double pitch = Constants.LL_ANGLE_DEG + vAngle;
		double distance = Math.tan(Math.toRadians(pitch)) / opposite;
		double yaw = MoUtil.wrapAngleDeg(pose.getRotation().getDegrees() - hAngle);

		Translation2d offset = new Translation2d(
			Math.cos(Math.toDegrees(yaw)),
			Math.sin(Math.toDegrees(yaw))).times(distance + (Constants.GOAL_DIAMETER_M * 0.5));
		
		SmartDashboard.putNumber("LL H Angle", hAngle);
		SmartDashboard.putNumber("LL V Angle", vAngle);
		SmartDashboard.putNumber("LL Last Dist", distance);
			
		if (this.lastLLPos == null) {
			this.lastLLPos = offset;
		} else {
			if (this.lastLLPos.getDistance(offset) <= 1.8) {
				this.llConfidence++;
			} else {
				this.llConfidence = 0;
			}
			this.lastLLPos = offset;
		}
		if (this.llConfidence > 3) {
			this.targetPose = pose.plus(new Transform2d(offset, new Rotation2d(0)).inverse());
			
			SmartDashboard.putString("Target Position", this.targetPose.getTranslation().toString());
		}
	}

	public Pose2d getPoseRelativeToTarget() {
		return this.driveSubsystem.getPose().relativeTo(this.targetPose);
	}

	// Limelight values are automatically put into a network
	// table, this is a wrapper around that
	public static class LimelightTableAdapter {
		private boolean hasTarget = false;
		private double hOffset = 0;
		private double vOffset = 0;
		private double targetArea = 0;

		private NetworkTable getTable() {
			return NetworkTableInstance.getDefault().getTable("limelight");
		}

		public void periodic() {
			NetworkTable table = this.getTable();

			this.hasTarget = table.getEntry("tv").getNumber(0).doubleValue() > 0;
			this.hOffset = table.getEntry("tx").getNumber(0).doubleValue();
			this.vOffset = table.getEntry("ty").getNumber(0).doubleValue();
			this.targetArea = table.getEntry("ta").getNumber(0).doubleValue();
		}

		public boolean hasTarget() {
			return this.hasTarget;
		}

		public void ifTarget(TargetConsumer action) {
			if (this.hasTarget()) {
				action.accept(this.hOffset, this.vOffset, this.targetArea);
			}
		}

		public void setLight(boolean on) {
			this.getTable().getEntry("ledMode").setNumber(on ? 3 : 1);
		}

		private static interface TargetConsumer {
			void accept(double h, double v, double area);
		}
	}
}
