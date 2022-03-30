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

	private Pose2d originPose;
	private Pose2d targetPose;
	private boolean hasFirstInit;

	public TargetingSubsystem(DriveSubsystem drive) {
		this.driveSubsystem = drive;

		resetOrigin();
		hasFirstInit = false;
	}

	public void resetOrigin() {
		this.originPose = this.driveSubsystem.getPose();
		double yaw = this.originPose.getRotation().getDegrees();
		Translation2d offset = new Translation2d(
			Math.cos(Math.toDegrees(-yaw)),
			Math.sin(Math.toDegrees(-yaw))).times(Constants.ROBOT_START_DISTANCE_FROM_GOAL_M);
		this.targetPose = this.originPose.plus(new Transform2d(offset, new Rotation2d(0)).inverse());
		
		SmartDashboard.putString("Target Position", this.targetPose.getTranslation().toString());
		this.hasFirstInit = true;
	}

	private double calculateTurnPower(double angleDiff, double range, double minPower, double falloff) {
		double coefficient = -(minPower / Math.pow(range, 4 * falloff));
		double polynomial = Math.pow(angleDiff - range, 2 * falloff) * Math.pow(angleDiff + range, 2 * falloff);
		double result = coefficient * polynomial + 1;

		if (angleDiff < 0) {
			result *= -1;
		}
		return result;
	}

	public void turnToTarget() {
		double power = this.calculateTurnPower(
			this.getTargetAngleDifference(), 180, 
			MoPrefs.TARGETING_MIN_PWR.get(), 
			MoPrefs.TARGETING_FALLOFF.get());

		if (!this.isAtTarget()) {
			this.driveSubsystem.driveDirect(power, -power);
		}
	}

	public boolean isAtTarget() {
		return Math.abs(this.getTargetAngleDifference()) < Constants.TARGETING_ANGLE_ERROR_DEG;
	}

	public double getTargetAngleDifference() {
		Pose2d pose = getPoseRelativeToTarget();
		double angle = MoUtil.wrapAngleDeg(-Math.toDegrees(Math.atan2(-pose.getTranslation().getY(), pose.getTranslation().getX())));
		double robotAngle = pose.getRotation().getDegrees();
		
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

		this.limelight.ifTarget(this::reOriginFromLimelight);
	}

	private void reOriginFromLimelight(double hAngle, double vAngle, double area) {
		double opposite = Constants.LL_GOAL_HEIGHT_CM - Constants.LL_HEIGHT_CM;
		double pitch = Constants.LL_ANGLE_DEG + vAngle;
		double distance = Math.sin(Math.toRadians(pitch)) / opposite;
		double yaw = MoUtil.wrapAngleDeg(this.driveSubsystem.getPose().getRotation().getDegrees() + hAngle);

		Translation2d offset = new Translation2d(
			Math.cos(Math.toDegrees(-yaw)),
			Math.sin(Math.toDegrees(-yaw))).times(distance + (Constants.GOAL_DIAMETER_M * 0.5));
		this.targetPose = this.originPose.plus(new Transform2d(offset, new Rotation2d(0)).inverse());
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
