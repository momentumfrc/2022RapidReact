package com.momentum4999.robot.commands;

import com.momentum4999.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIdleCommand extends Command {
    private final ShooterSubsystem shooter;

    private static final double SHOOTER_CLOSED_POS = 0.7;
    private static final double SHOOTER_CLOSE_POWER = 0.2;

    public ShooterIdleCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (shooter.hood.getEncoder().getPosition() > SHOOTER_CLOSED_POS) {
            shooter.hood.set(-SHOOTER_CLOSE_POWER);
        } else {
            shooter.hood.set(0);
        }
    }
}
