package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private final MotorController indexer = new VictorSP(Components.INDEXER);
	private final MotorController shooter = new Spark(Components.SHOOTER);
}
