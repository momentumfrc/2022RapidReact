package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.AnimationSequence;
import com.momentum4999.robot.util.AnimationSequence.AnimationSequenceMember;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.LEDUtils;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static final Dimensionless LED_BRIGHTNESS = Units.Percent.of(80);

    private static final int LED_LENGTH = 240;
    private static final Distance LED_SPACING = Units.Meters.of(1 / 60.0);

    private final LEDPattern defaultPattern;

    private final AddressableLED neopixels;
    private final AddressableLEDBuffer ledBuffer;

    public LEDSubsystem() {
        neopixels = new AddressableLED(Components.LEDS);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        neopixels.setLength(LED_LENGTH);
        neopixels.start();

        Color[] rainbowTails = LEDUtils.getColorTails(LEDUtils.RAINBOW, Color.kBlack, 12, 20);
        Color[] momentumTails = LEDUtils.getColorTails(
                new Color[] {LEDUtils.MOMENTUM_BLUE, LEDUtils.MOMENTUM_PURPLE}, Color.kBlack, 24, 32);

        defaultPattern = new AnimationSequence(
                        new AnimationSequenceMember(
                                LEDUtils.scrollBuffer(rainbowTails, Units.MetersPerSecond.of(0.833), LED_SPACING),
                                Units.Seconds.of(5)),
                        new AnimationSequenceMember(
                                LEDUtils.scrollBuffer(
                                        LEDUtils.getSmearedColors(LEDUtils.RAINBOW, 16),
                                        Units.MetersPerSecond.of(1.11),
                                        LED_SPACING),
                                Units.Seconds.of(3)),
                        new AnimationSequenceMember(
                                LEDUtils.scrollBuffer(momentumTails, Units.MetersPerSecond.of(0.833), LED_SPACING),
                                Units.Seconds.of(5)))
                .atBrightness(LED_BRIGHTNESS);
        setDefaultCommand(runPattern(defaultPattern));
    }

    public Command runPattern(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(ledBuffer), this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        neopixels.setData(ledBuffer);
    }

    /* TODO: Reimplement Stack animation in new LED library

        new AnimationSequence.AnimationSequenceMember(new Stack(20, 60, rainbowcolors), 1500)
    */
}
