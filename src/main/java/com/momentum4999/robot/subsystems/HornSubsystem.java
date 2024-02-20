package com.momentum4999.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HornSubsystem extends SubsystemBase {
    private DigitalOutput horn = new DigitalOutput(0);

    public void honk() {
        horn.set(true);
    }

    public void unhonk() {
        horn.set(false);
    }
}
