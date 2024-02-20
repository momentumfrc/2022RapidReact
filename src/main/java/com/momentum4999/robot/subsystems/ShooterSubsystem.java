package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, MotorType.kBrushless);
    private final CANSparkMax shooterA = new CANSparkMax(Components.SHOOTER_A, MotorType.kBrushless);
    private final CANSparkMax shooterB = new CANSparkMax(Components.SHOOTER_B, MotorType.kBrushless);
    public final CANSparkMax hood = new CANSparkMax(Components.HOOD, MotorType.kBrushless);

    private final RelativeEncoder shooterAEncoder = shooterA.getEncoder();
    private final RelativeEncoder shooterBEncoder = shooterB.getEncoder();
    private final SparkPIDController shootAPidController = shooterA.getPIDController();

    private ShootStep currentShootStep = ShootStep.OPEN_HOOD;

    public ShooterSubsystem() {
        this.indexer.setInverted(true);
        this.shooterB.setInverted(true);
        this.shooterB.follow(shooterA, true);
        this.hood.setInverted(true);
    }

    public boolean shooterUpToSpeed() {
        double min = MoPrefs.SHOOTER_SETPOINT.get() - MoPrefs.SHOOTER_TARGET_ERROR.get();
        return this.shooterAEncoder.getVelocity() > min;
    }

    public boolean hoodFullyOpen() {
        return this.hood.getEncoder().getPosition() >= this.calcHoodTarget();
    }

    public void runIndexer(boolean rev) {
        this.indexer.set((rev ? -0.5 : 1) * MoPrefs.INDEXER_SETPOINT.get());
    }

    public void idleIndexer() {
        this.indexer.stopMotor();
    }

    public void runShooter() {
        this.shootAPidController.setReference(MoPrefs.SHOOTER_SETPOINT.get(), CANSparkMax.ControlType.kVelocity, 0);
    }

    public void retractShooter() {
        this.shooterA.set(-0.2);
    }

    private double calcHoodTarget() {
        return MoPrefs.HOOD_DISTANCE_TEST.get();
    }

    public void openHood() {
        if (!this.hoodFullyOpen()) {
            this.hood.set(0.4);
        } else {
            this.hood.set(0);
        }
    }

    public void runActive() {
        if (currentShootStep == ShootStep.OPEN_HOOD) {
            if (this.hoodFullyOpen()) {
                currentShootStep = ShootStep.RAMP_UP;
            }
        } else if (currentShootStep == ShootStep.RAMP_UP) {
            if (this.shooterUpToSpeed()) {
                currentShootStep = ShootStep.SHOOT;
            }
        } else if (currentShootStep == ShootStep.SHOOT) {
            if (!this.shooterUpToSpeed() || !this.hoodFullyOpen()) {
                currentShootStep = ShootStep.OPEN_HOOD;
            }
        } else {
            throw new IllegalStateException("Unknown shooterState " + currentShootStep.name());
        }

        if (currentShootStep == ShootStep.OPEN_HOOD) {
            this.openHood();
            this.idleIndexer();
            this.idleShooter();
        } else if (currentShootStep == ShootStep.RAMP_UP) {
            this.openHood();
            this.idleIndexer();
            this.runShooter();
        } else if (currentShootStep == ShootStep.SHOOT) {
            this.openHood();
            this.runIndexer(false);
            this.runShooter();
        } else {
            throw new IllegalStateException("Unknown shooterState " + currentShootStep.name());
        }
    }

    public void stop() {
        idle();
        this.currentShootStep = ShootStep.OPEN_HOOD;
    }

    public void idleShooter() {
        this.shooterA.stopMotor();
        this.shooterB.stopMotor();
    }

    public void idle() {
        idleIndexer();
        idleShooter();
    }

    @Override
    public void periodic() {
        MoShuffleboard.putNumber("Flywheel Motor A Velocity", shooterAEncoder.getVelocity());
        MoShuffleboard.putNumber("Flywheel Motor B Velocity", shooterBEncoder.getVelocity());
        MoShuffleboard.putNumber("Current Hood Dist", this.hood.getEncoder().getPosition());
        MoShuffleboard.putString("Shooter State", this.currentShootStep.name());

        // Shooter B is reversed and set to follow
        this.shootAPidController.setP(MoPrefs.SHOOTER_KP.get());
        this.shootAPidController.setI(MoPrefs.SHOOTER_KI.get());
        this.shootAPidController.setD(MoPrefs.SHOOTER_KD.get());
        this.shootAPidController.setFF(MoPrefs.SHOOTER_KFF.get());
        this.shootAPidController.setIZone(MoPrefs.SHOOTER_KIZONE.get());
        this.shootAPidController.setDFilter(0);
    }

    public enum ShootStep {
        OPEN_HOOD,
        RAMP_UP,
        SHOOT;
    }
}
