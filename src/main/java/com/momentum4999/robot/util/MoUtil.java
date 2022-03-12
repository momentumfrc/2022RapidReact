package com.momentum4999.robot.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.NoSuchElementException;
import java.util.stream.Stream;

import com.momentum4999.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public final class MoUtil {
	private MoUtil() {}

	public static DoubleSolenoid.Value getSolenoidOpposite(DoubleSolenoid.Value value) {
		switch (value) {
			case kForward: return DoubleSolenoid.Value.kReverse;
			case kReverse: return DoubleSolenoid.Value.kForward;
			default:       return DoubleSolenoid.Value.kOff;
		}
	}

	public static double wrapAngleDeg(double angle) {
		angle = angle % 360;

		if (angle > 180) {
			angle -= 180;
			angle = -1 * (180 - angle);
		} else if (angle < -180) {
			angle += 180;
			angle = -1 * (180 + angle);
		}

		return angle;
	}

	public static String readResourceFile(String file) {
		try (InputStream in = Robot.class.getClassLoader().getResourceAsStream(file)) {
			if (in == null) {
				System.err.println("Resource file '"+file+"' doesn't exist!");
				return "";
			}

			Stream<String> data = new BufferedReader(new InputStreamReader(in)).lines();
			try {
				return data.reduce((a, b) -> a + "\n" + b).get();
			} catch (NoSuchElementException ignored) {}
		} catch (IOException ex) {
			System.err.println("Failed to read resource file '"+file+"': " + ex.getMessage());
		}

		return "";
	}
}
