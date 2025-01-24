package com.momentum4999.robot.commands;

import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.util.MoPrefs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    private final MoInput input;

    public DriveCommand(DriveSubsystem drive, MoInput input) {
        this.drive = drive;
        this.input = input;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    private double applyInputTransforms(double v) {
        v = MathUtil.applyDeadband(v, MoPrefs.DRIVE_DEADBAND.get());
        v = Math.copySign(Math.pow(Math.abs(v), MoPrefs.DRIVE_CURVE.get()), v);
        v = v * MathUtil.clamp(MoPrefs.DRIVE_THROTTLE.get(), 0, 1);
        return v;
    }

    @Override
    public void execute() {
        double mr = applyInputTransforms(input.getMoveRequest());
        double tr = applyInputTransforms(input.getTurnRequest());

        this.drive.arcadeDrive(mr, tr);
    }

    @Override
    public void end(boolean interrupted) {
        this.drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
