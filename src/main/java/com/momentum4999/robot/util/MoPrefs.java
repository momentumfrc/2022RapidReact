package com.momentum4999.robot.util;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MoPrefs {
	private static MoPrefs instance = null;

	private static final String tableKey = "Preferences";
	private static final Set<Runnable> initListeners = new HashSet<>();
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
		"ShooterSetpoint", 4200);
	public static final Pref<Double> HOOD_SETPOINT = doublePref(
		"HoodSetpoint", 0.85);
	public static final Pref<Double> HOOD_DISTANCE_TEST = doublePref(
		"HoodDistanceTest", 18);
	public static final Pref<Double> CLIMBER_RAISE_SETPOINT = doublePref(
		"ClimbRaiseSetpoint", 1);
	public static final Pref<Double> CLIMBER_ADJUST_SETPOINT = doublePref(
		"ClimbAdjustSetpoint", 1);
	public static final Pref<Double> CLIMB_HEIGHT = doublePref(
		"ClimbHeight", 400);
	public static final Pref<Double> CLIMB_ADJUST_MIN = doublePref(
		"ClimbAdjustMin", -20);	
	public static final Pref<Double> CLIMB_ADJUST_MAX = doublePref(
		"ClimbAdjustMax", 120);
	public static final Pref<Double> CLIMB_ADJUST_LIM = doublePref(
		"ClimbAdjustLim", 75);
	public static final Pref<Double> TARGETING_MIN_PWR = doublePref(
		"TargetingMinPower", 0.4);
	public static final Pref<Double> TARGETING_FALLOFF = doublePref(
		"TargetingFalloff", 1);
	public static final Pref<Double> TARGETING_ANGLE_ERROR = doublePref(
		"TargetingAngleError", 8.5);
	public static final Pref<Double> SHOOTER_TARGET_ERROR = doublePref(
		"ShooterTargetError", 200);
	public static final Pref<Double> SHOOTER_KP = doublePref(
		"ShooterkP", 0.0003);
	public static final Pref<Double> SHOOTER_KFF = doublePref(
		"ShooterkFF", 0.00002935);3
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

		initListeners.forEach(Runnable::run);
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

			initListeners.add(this::init);
		}

		private void init() {
			this.set(defaultValue);
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
