package com.momentum4999.robot.subsystems;

import com.momentum4999.robot.util.Components;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax indexer = new CANSparkMax(Components.INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax shooter = new CANSparkMax(Components.SHOOTER, CANSparkMaxLowLevel.MotorType.kBrushless);
}
