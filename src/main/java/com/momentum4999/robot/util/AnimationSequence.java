package com.momentum4999.robot.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import java.util.Objects;

public class AnimationSequence implements LEDPattern {

    public record AnimationSequenceMember(LEDPattern pattern, Time showTime) {}

    private final AnimationSequenceMember[] members;
    private long startTime;
    private int currentIdx = 0;

    public AnimationSequence(AnimationSequenceMember... members) {
        Objects.requireNonNull(members);
        this.members = members;
        startTime = WPIUtilJNI.now();
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        AnimationSequenceMember curr = members[currentIdx];
        long currentTime = WPIUtilJNI.now();
        long showTime = (long) curr.showTime.in(Units.Microsecond);

        if (currentTime - startTime > showTime) {
            startTime = currentTime;
            currentIdx = (currentIdx + 1) % members.length;
            curr = members[currentIdx];
        }

        curr.pattern.applyTo(reader, writer);
    }
}
