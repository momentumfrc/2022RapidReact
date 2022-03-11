package com.momentum4999.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MoShuffleboard {
	public static ShuffleboardTab matchTab() {
		return Shuffleboard.getTab("Match");
	}
}
