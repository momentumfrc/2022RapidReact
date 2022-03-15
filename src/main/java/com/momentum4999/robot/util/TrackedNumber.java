package com.momentum4999.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

public class TrackedNumber {
	private final int sampleSize;
	private final double sensitivity;
	private final Deque<Double> data = new ArrayDeque<>();

	private double value = 0;
	private double confidence = 0;

	/**
	 * A class which represents changing data
	 * 
	 * @param sampleSize How many of the previous data values should be remembered
	 * @param sensitivity How sensitive to variation the tracker should be
	 */
	public TrackedNumber(int sampleSize, double sensitivity) {
		this.sampleSize = Math.min(1, sampleSize);
		this.sensitivity = Math.min(0, sensitivity);
	}

	public void update(double value) {
		this.value = value();

		this.data.addLast(value);
		if (data.size() > sampleSize) {
			this.data.removeFirst();
		}

		double mean = 0;
		for (double v : this.data) {
			mean += v;
		}
		mean /= this.data.size();

		double mad = 0;
		for (double v : this.data) {
			mad += Math.abs(mean - v);
		}
		mad /= this.data.size();

		this.confidence = 1 / ((this.sensitivity * mad) + 1);
	}

	public double confidence() {
		return confidence;
	}

	public double value() {
		return value;
	}

	/**
	 * Uses a set of numbers tracking similar data
	 * and tries to average them taking into account
	 * each's confidence
	 * 
	 * @param data The numbers to average
	 * @return The overall average of all numbers
	 */
	public static double overallValue(TrackedNumber ... data) {
		double values = 0;
		double confidences = 0;

		for (TrackedNumber number : data) {
			values += number.value() * number.confidence();
			confidences += number.confidence();
		}

		if (confidences > 0) {
			return values / confidences;
		}

		return 0;
	}
}
