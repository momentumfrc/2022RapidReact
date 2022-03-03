package com.momentum4999.robot.util;

import java.util.function.BiConsumer;
import java.util.function.BiFunction;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MoPrefs {
	private static MoPrefs instance = null;

	private static final String tableKey = "Preferences";
	private final NetworkTable table;

	// Number preferences are defined like so:
	// public static final Pref<Double> EX_NUMBER = doublePref("Example", <default value>);

	// Text preferences are defined like so:
	// public static final Pref<String> EX_TEXT = doublePref("Example", "default value");

	// ----------------- Define Preferences Here -----------------
	public static final Pref<Double> INTAKE_ROLLER_SETPOINT = doublePref(
		"IntakeRollerSetpoint", 1);
	public static final Pref<Double> INDEXER_SETPOINT = doublePref(
		"IndexerSetpoint", 1);
	public static final Pref<Double> SHOOTER_SETPOINT = doublePref(
		"ShooterSetpoint", 1);
	public static final Pref<Double> SHOOTER_TARGET = doublePref(
		"ShooterTarget", 4800);
	public static final Pref<Double> SHOOTER_TARGET_ERROR = doublePref(
		"ShooterTargetError", 280);
	// -----------------------------------------------------------

	private MoPrefs() {
		this.table = NetworkTableInstance.getDefault().getTable(tableKey);
		this.table.getEntry(".type").setString("RobotPreferences");
		this.table.addEntryListener((table, key, entry, value, flags) -> entry.setPersistent(), 
			EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
	}

	public NetworkTableEntry getEntry(String key) {
		return this.table.getEntry(key);
	}

	public static MoPrefs get() {
		if (instance == null) {
			throw new RuntimeException("Tried to access MoPrefs too early!");
		}

		return instance;
	}

	public static void init() {
		if (instance != null) {
			throw new RuntimeException("Tried to initialize MoPrefs twice!");
		}

		instance = new MoPrefs();
	}

	public static final class Pref<T> {
		public final String key;
		private final T defaultValue;

		// A function that gets this pref's value from a NetworkTableEntry
		private final BiFunction<NetworkTableEntry, T, T> getter;
		// A function that sets this pref's value within a NetworkTableEntry
		private final BiConsumer<NetworkTableEntry, T> setter;

		public Pref(String key, T defaultValue, 
					BiFunction<NetworkTableEntry, T, T> getter, BiConsumer<NetworkTableEntry, T> setter) {
			this.key = key;
			this.defaultValue = defaultValue;
			this.getter = getter;
			this.setter = setter;
		}

		public T get() {
			MoPrefs prefs = MoPrefs.get();
			return this.getter.apply(prefs.getEntry(this.key), this.defaultValue);
		}

		public void set(T value) {
			MoPrefs prefs = MoPrefs.get();
			this.setter.accept(prefs.getEntry(this.key), value);
		}
	}

	private static Pref<String> stringPref(String key, String defaultVal) {
		return new Pref<>(key, defaultVal, NetworkTableEntry::getString, NetworkTableEntry::setString);
	}

	private static Pref<Double> doublePref(String key, double defaultVal) {
		return new Pref<>(key, defaultVal, NetworkTableEntry::getDouble, NetworkTableEntry::setDouble);
	}
}
