package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.MoPrefs;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HornSubsystem extends SubsystemBase {
    private Relay horn = new Relay(0, Relay.Direction.kForward);

    private PWMVictorSPX hornMotor = new PWMVictorSPX(2);

    private SlewRateLimiter limiter;

    public HornSubsystem() {
        MoPrefs.HORN_RAMP_TIME.subscribe(
                ramp -> {
                    limiter = new SlewRateLimiter(1 / ramp);
                },
                true);
    }

    private void set(double value) {
        hornMotor.set(limiter.calculate(value));
    }

    public void honk() {
        // horn.set(Relay.Value.kOn);

        set(MoPrefs.HORN_SPD.get());
        horn.set(Value.kOn);
    }

    public void unhonk() {
        // horn.set(Relay.Value.kOff);
        set(0);
        horn.set(Value.kOff);
    }
}
