package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoPrefs;
import com.momentum4999.robot.util.MoShuffleboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax indexer = new SparkMax(Components.INDEXER, MotorType.kBrushless);
    private final SparkMax shooterA = new SparkMax(Components.SHOOTER_A, MotorType.kBrushless);
    private final SparkMax shooterB = new SparkMax(Components.SHOOTER_B, MotorType.kBrushless);
    public final SparkMax hood = new SparkMax(Components.HOOD, MotorType.kBrushless);

    private final RelativeEncoder shooterAEncoder = shooterA.getEncoder();
    private final RelativeEncoder shooterBEncoder = shooterB.getEncoder();
    private final SparkClosedLoopController shootAPidController = shooterA.getClosedLoopController();

    private ShootStep currentShootStep = ShootStep.OPEN_HOOD;

    public ShooterSubsystem() {

        var shooterAConfig = new SparkMaxConfig();
        shooterAConfig.inverted(false);
        shooterAConfig
                .closedLoop
                .p(MoPrefs.SHOOTER_KP.get())
                .i(MoPrefs.SHOOTER_KI.get())
                .d(MoPrefs.SHOOTER_KD.get())
                .velocityFF(MoPrefs.SHOOTER_KFF.get())
                .iZone(MoPrefs.SHOOTER_KIZONE.get())
                .dFilter(0);
        this.shooterA.configure(shooterAConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.indexer.configure(
                new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        this.shooterB.configure(
                new SparkMaxConfig().follow(shooterA, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        this.hood.configure(
                new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        MoPrefs.SHOOTER_KP.subscribe(
                p -> {
                    var config = new SparkMaxConfig();
                    config.closedLoop.p(p, ClosedLoopSlot.kSlot0);
                    this.shooterA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                false);

        MoPrefs.SHOOTER_KI.subscribe(
                i -> {
                    var config = new SparkMaxConfig();
                    config.closedLoop.i(i, ClosedLoopSlot.kSlot0);
                    this.shooterA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                false);

        MoPrefs.SHOOTER_KD.subscribe(
                d -> {
                    var config = new SparkMaxConfig();
                    config.closedLoop.d(d, ClosedLoopSlot.kSlot0);
                    this.shooterA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                false);

        MoPrefs.SHOOTER_KFF.subscribe(
                ff -> {
                    var config = new SparkMaxConfig();
                    config.closedLoop.velocityFF(ff, ClosedLoopSlot.kSlot0);
                    this.shooterA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                false);

        MoPrefs.SHOOTER_KIZONE.subscribe(
                iZone -> {
                    var config = new SparkMaxConfig();
                    config.closedLoop.iZone(iZone, ClosedLoopSlot.kSlot0);
                    this.shooterA.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                false);
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
        this.shootAPidController.setReference(
                MoPrefs.SHOOTER_SETPOINT.get(), SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
    }

    public enum ShootStep {
        OPEN_HOOD,
        RAMP_UP,
        SHOOT;
    }
}
