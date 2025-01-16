package com.momentum4999.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.momentum4999.robot.util.Components;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private static final int LED_LENGTH = 240;
    private static final Distance LED_SPACING = Units.Meters.of(1 / 60.0);

    private final LEDPattern defaultPattern =
            LEDPattern.rainbow(255, (int) (0.8 * 255)).scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LED_SPACING);

    private final AddressableLED neopixels;
    private final AddressableLEDBuffer ledBuffer;

    public LEDSubsystem() {
        neopixels = new AddressableLED(Components.LEDS);
        ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        neopixels.setLength(LED_LENGTH);
        neopixels.start();

        setDefaultCommand(runPattern(defaultPattern));
    }

    public Command runPattern(LEDPattern pattern) {
        return Commands.run(() -> pattern.applyTo(ledBuffer), this).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        neopixels.setData(ledBuffer);
    }

    /* TODO: Reimplement old animations in new LED library

    private static Color[] rainbowcolors = {
        new Color(72, 21, 170),
        new Color(55, 131, 255),
        new Color(77, 233, 76),
        new Color(255, 238, 0),
        new Color(255, 140, 0),
        new Color(246, 0, 0)
    };

    Color[] rainbowTails = ColorTools.getColorTails(rainbowcolors, Color.BLACK, 12, 20);
    Color[] momentumTails =
            ColorTools.getColorTails(new Color[] {Color.MOMENTUM_BLUE, Color.MOMENTUM_PURPLE}, Color.BLACK, 24, 32);

    Animation mainAnimation = new AnimationSequence(new AnimationSequence.AnimationSequenceMember[] {
        new AnimationSequence.AnimationSequenceMember(new Snake(20, rainbowTails), 5000),
        new AnimationSequence.AnimationSequenceMember(
                new Snake(15, ColorTools.getSmearedColors(rainbowcolors, 16)), 1500),
        new AnimationSequence.AnimationSequenceMember(new Snake(20, momentumTails), 5000),
        new AnimationSequence.AnimationSequenceMember(new Stack(20, 60, rainbowcolors), 1500)
    });
    */
}
