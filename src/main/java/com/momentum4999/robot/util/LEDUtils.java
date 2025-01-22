package com.momentum4999.robot.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDUtils {

    public static Color MOMENTUM_BLUE = new Color(6, 206, 255);
    public static Color MOMENTUM_PURPLE = new Color(159, 1, 255);

    public static Color[] RAINBOW = {
        new Color(246, 0, 0),
        new Color(255, 140, 0),
        new Color(255, 238, 0),
        new Color(77, 233, 6),
        new Color(55, 131, 255),
        new Color(72, 21, 170)
    };

    record GradientStop(int pos, Color color) {}

    public static LEDPattern repeatBuffer(Color[] buffer) {
        return (reader, writer) -> {
            for (int i = 0; i < reader.getLength(); i++) {
                writer.setLED(i, buffer[i % buffer.length]);
            }
        };
    }

    public static LEDPattern scrollBuffer(Color[] buffer, LinearVelocity velocity, Distance ledSpacing) {
        var metersPerMicro = velocity.in(Units.Meters.per(Units.Microsecond));
        var microsPerLED = (int) (ledSpacing.in(Units.Meters) / metersPerMicro);

        return (ledReader, ledWriter) -> {
            long now = WPIUtilJNI.now();

            // every step in time that's a multiple of microsPerLED will increment the offset by 1
            var offset = (int) (now / microsPerLED);

            for (int i = 0; i < ledReader.getLength(); i++) {
                ledWriter.setLED(i, buffer[Math.floorMod(i + offset, buffer.length)]);
            }
        };
    }

    public static Color[] getGradient(int gradientSize, GradientStop[] stops) {
        if (stops[0].pos() != 0) {
            throw new IllegalArgumentException("Missing initial gradient stop");
        }
        for (int i = 0; i < stops.length; ++i) {
            GradientStop stop = stops[i];
            if (stop.pos() < 0 || stop.pos() >= gradientSize) {
                throw new IllegalArgumentException("Gradient stop pos is out of bounds");
            }
            if (i < stops.length - 1) {
                GradientStop nextStop = stops[i + 1];
                if (stop.pos() >= nextStop.pos()) {
                    throw new IllegalArgumentException("Gradient stop positions must always increase");
                }
            }
        }

        if (stops[stops.length - 1].pos() < gradientSize - 1) {
            // If a final gradient stop is not specified, we guess it
            // by wrapping-around back to the initial gradient stop

            GradientStop[] newStops = new GradientStop[stops.length + 1];
            System.arraycopy(stops, 0, newStops, 0, stops.length);

            GradientStop lastDefinedStop = stops[stops.length - 1];

            Color srcColor = lastDefinedStop.color();
            Color targetColor = stops[0].color();

            int finalDiffLen = gradientSize - lastDefinedStop.pos();

            double lastStopFrac = (finalDiffLen - 1) / ((double) finalDiffLen);

            double deltaR, deltaG, deltaB;
            deltaR = (targetColor.red - srcColor.red) * lastStopFrac;
            deltaG = (targetColor.green - srcColor.green) * lastStopFrac;
            deltaB = (targetColor.blue - srcColor.blue) * lastStopFrac;

            newStops[newStops.length - 1] = new GradientStop(
                    gradientSize - 1,
                    new Color(srcColor.red + deltaR, srcColor.green + deltaG, srcColor.blue + deltaB));

            stops = newStops;
        }

        if (stops[stops.length - 1].pos() != gradientSize - 1) {
            throw new IllegalArgumentException("Missing final gradient stop");
        }

        Color[] gradientBuff = new Color[gradientSize];
        Color currStopC, nextStopC;
        double deltaR, deltaG, deltaB;
        for (int i = 0; i < stops.length - 1; ++i) {
            GradientStop currStop = stops[i];
            GradientStop nextStop = stops[i + 1];
            currStopC = currStop.color();
            nextStopC = nextStop.color();
            double stopPixelDiff = nextStop.pos() - currStop.pos();
            deltaR = (nextStopC.red - currStopC.red) / stopPixelDiff;
            deltaG = (nextStopC.green - currStopC.green) / stopPixelDiff;
            deltaB = (nextStopC.blue - currStopC.blue) / stopPixelDiff;
            for (int j = currStop.pos(); j < nextStop.pos(); ++j) {
                int pixelDiff = j - currStop.pos();
                Color currColor = new Color(
                        currStopC.red + (deltaR * pixelDiff),
                        currStopC.green + (deltaG * pixelDiff),
                        currStopC.blue + (deltaB * pixelDiff));
                gradientBuff[j] = currColor;
            }
        }
        GradientStop finalStop = stops[stops.length - 1];
        gradientBuff[finalStop.pos()] = finalStop.color;

        return gradientBuff;
    }

    public static Color[] getSmearedColors(Color[] colors, int smear) {
        int gradientSize = colors.length * smear;

        GradientStop[] stops = new GradientStop[colors.length];
        for (int i = 0; i < colors.length; ++i) {
            stops[i] = new GradientStop(i * smear, colors[i]);
        }

        return getGradient(gradientSize, stops);
    }

    public static Color[] getColorTails(Color[] colors, Color background, int tailLen, int spacing) {
        int totalLength = 1;
        int stopsPerColor = 2;
        if (tailLen == 0) {
            tailLen = 1;
        }
        totalLength += tailLen;
        if (spacing > 0) {
            stopsPerColor += 1;
            totalLength += spacing;
        }

        GradientStop[] stops = new GradientStop[colors.length * stopsPerColor];
        for (int i = 0; i < colors.length; ++i) {
            stops[i * stopsPerColor] = new GradientStop(i * totalLength, colors[i]);

            stops[(i * stopsPerColor) + 1] = new GradientStop((i * totalLength) + tailLen, background);

            if (spacing > 0) {
                stops[(i * stopsPerColor) + 2] = new GradientStop((i * totalLength) + tailLen + spacing, background);
            }
        }

        return getGradient(colors.length * (1 + tailLen + spacing), stops);
    }

    private LEDUtils() {
        throw new UnsupportedOperationException("LEDUtils is a static class");
    }
}
