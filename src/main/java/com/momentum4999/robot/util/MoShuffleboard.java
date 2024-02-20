package com.momentum4999.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class MoShuffleboard {
    private static final String DEFAULT_TAB = "MoDashboard";

    static class LayoutDirective {
        private final ShuffleboardContainer container;
        private final Consumer<SimpleWidget> doLayout;

        public LayoutDirective(String tabName, Consumer<SimpleWidget> doLayout) {
            this(Shuffleboard.getTab(tabName), doLayout);
        }

        public LayoutDirective(ShuffleboardContainer container, Consumer<SimpleWidget> doLayout) {
            this.container = container;
            this.doLayout = doLayout;
        }

        public ShuffleboardContainer getContainer() {
            return container;
        }

        public void doLayout(SimpleWidget widget) {
            this.doLayout.accept(widget);
        }
    }

    private static HashMap<String, LayoutDirective> directives = new HashMap<>();
    private static HashMap<String, GenericEntry> entries = new HashMap<>();

    public static ShuffleboardTab matchTab() {
        return Shuffleboard.getTab("Match");
    }

    static {
        // Drive
        ShuffleboardLayout driveLayout = Shuffleboard.getTab("MoSystems")
                .getLayout("Drive", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(2, 0)
                .withProperties(Map.of("Label Position", "Top"));
        directives.put("Average Distance", new LayoutDirective(driveLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(0, 0)));
        directives.put("Left Distance", new LayoutDirective(driveLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(0, 1)));
        directives.put("Right Distance", new LayoutDirective(driveLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(1, 1)));
        directives.put(
                "Gyro Degrease" /* WTF is that spelling? */,
                new LayoutDirective(driveLayout, (widget) -> widget.withWidget(BuiltInWidgets.kTextView)
                        .withPosition(1, 0)));

        // Shooter
        ShuffleboardLayout shootLayout = Shuffleboard.getTab("MoSystems")
                .getLayout("Shooter", BuiltInLayouts.kGrid)
                .withSize(2, 3)
                .withPosition(4, 0)
                .withProperties(Map.of("Label Position", "Top"));
        directives.put("Flywheel Motor A Velocity", new LayoutDirective(shootLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(0, 0)));
        directives.put("Flywheel Motor B Velocity", new LayoutDirective(shootLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kNumberBar)
                .withPosition(1, 0)));
        directives.put("Current Hood Dist", new LayoutDirective(shootLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(0, 1)));
        directives.put("Shooter State", new LayoutDirective(shootLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kTextView)
                .withPosition(1, 1)));
        directives.put("Full of Ball", new LayoutDirective(shootLayout, (widget) -> widget.withWidget(
                        BuiltInWidgets.kBooleanBox)
                .withPosition(0, 2)));

        Shuffleboard.selectTab("MoSystems");
    }

    private static void putValue(String key, Object value) {
        if (entries.containsKey(key)) {
            entries.get(key).setValue(value);
            return;
        }
        if (!directives.containsKey(key)) {
            GenericEntry entry =
                    Shuffleboard.getTab(DEFAULT_TAB).add(key, value).getEntry();
            entries.put(key, entry);
            return;
        }

        LayoutDirective dir = directives.get(key);
        ShuffleboardContainer cont = dir.getContainer();
        SimpleWidget widget = cont.add(key, value);
        dir.doLayout(widget);
        entries.put(key, widget.getEntry());
    }

    public static void putBoolean(String key, boolean value) {
        putValue(key, value);
    }

    public static void putNumber(String key, double value) {
        putValue(key, value);
    }

    public static void putString(String key, String value) {
        putValue(key, value);
    }

    public static void putBooleanArray(String key, boolean[] value) {
        putValue(key, value);
    }

    public static void putNumberArray(String key, double[] value) {
        putValue(key, value);
    }

    public static void putStringArray(String key, String[] value) {
        putValue(key, value);
    }

    public static void putRaw(String key, byte[] value) {
        putValue(key, value);
    }
}
