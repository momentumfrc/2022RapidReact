package com.momentum4999.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;
import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

public class MoPrefs {
    public static final Pref<Double> INDEXER_SETPOINT = doublePref("IndexerSetpoint", 1);
    public static final Pref<Double> SHOOTER_SETPOINT = doublePref("ShooterSetpoint", 4200);
    public static final Pref<Double> HOOD_DISTANCE_TEST = doublePref("HoodDistanceTest", 18);
    public static final Pref<Double> SHOOTER_TARGET_ERROR = doublePref("ShooterTargetError", 200);
    public static final Pref<Double> SHOOTER_KP = doublePref("ShooterkP", 0.00008);
    public static final Pref<Double> SHOOTER_KI = doublePref("ShooterkI", 0.0003);
    public static final Pref<Double> SHOOTER_KD = doublePref("ShooterkD", 0.0003);
    public static final Pref<Double> SHOOTER_KFF = doublePref("ShooterkFF", 0.00002935);

    public static final Pref<Double> SHOOTER_KIZONE = doublePref("ShooterIZone", 0.1);

    public static final Pref<Double> HORN_SPD = doublePref("Horn Speed", 1);
    public static final Pref<Double> HORN_RAMP_TIME = doublePref("Horn Ramp", 0.5);
    // -----------------------------------------------------------

    public final class Pref<T> {
        public final String key;
        private Function<NetworkTableValue, T> getter;
        private BiFunction<NetworkTableEntry, T, Boolean> setter;

        private final NetworkTableEntry entry;

        private Consumer<T> subscriber = null;

        public Pref(
                String key,
                T defaultValue,
                Function<NetworkTableValue, T> getter,
                BiFunction<NetworkTableEntry, T, Boolean> setter) {
            this.key = key;
            this.getter = getter;
            this.setter = setter;

            this.entry = table.getEntry(key);
            this.entry.setDefaultValue(defaultValue);
            this.entry.setPersistent();
        }

        public T get() {
            return getter.apply(entry.getValue());
        }

        public void set(T value) {
            setter.apply(entry, value);
        }

        public void subscribe(Consumer<T> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<T> consumer, boolean notifyImmediately) {
            if (subscriber != null) {
                subscriber = subscriber.andThen(consumer);
            } else {
                subscriber = consumer;
                entry.getInstance()
                        .addListener(
                                entry,
                                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                                (e) -> consumer.accept(getter.apply(e.valueData.value)));
            }

            if (notifyImmediately) {
                consumer.accept(this.get());
            }
        }
    }

    private static MoPrefs instance;
    private NetworkTable table;
    private StringPublisher typePublisher;

    private static MoPrefs getInstance() {
        if (instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    private MoPrefs() {
        table = NetworkTableInstance.getDefault().getTable("Preferences");
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("RobotPreferences");
    }

    private static Pref<Double> doublePref(String key, double defaultValue) {
        return getInstance().new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }
}
