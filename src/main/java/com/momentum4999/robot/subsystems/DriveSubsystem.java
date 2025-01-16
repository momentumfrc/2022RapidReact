package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final MotorController left = new VictorSP(Components.DR_LEFT);
    private final MotorController right = new VictorSP(Components.DR_RIGHT);

    private final DifferentialDrive driveTrain = new DifferentialDrive(left, right);

    public DriveSubsystem() {
        this.driveTrain.setDeadband(0);

        this.right.setInverted(true);
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
}
