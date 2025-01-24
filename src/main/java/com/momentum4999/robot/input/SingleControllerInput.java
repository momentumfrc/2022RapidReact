package com.momentum4999.robot.input;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControllerInput implements MoInput {

    private XboxController f310;

    public SingleControllerInput(XboxController f310) {
        this.f310 = f310;
    }

    @Override
    public double getMoveRequest() {
        return -1 * f310.getRawAxis(1);
    }

    @Override
    public double getTurnRequest() {
        return f310.getRawAxis(4);
    }

    @Override
    public boolean getRunShooter() {
        return f310.getBButton();
    }

    @Override
    public boolean getHonkHorn() {
        return f310.getAButton();
    }
}
