// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import java.util.Map;

import com.momentum4999.robot.commands.AutoDriveCommand;
import com.momentum4999.robot.commands.RunClimberCommand;
import com.momentum4999.robot.commands.ShooterActiveCommand;
import com.momentum4999.robot.commands.ShooterIdleCommand;
import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.commands.ZeroClimberCommand;
import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.input.SingleControllerInput;
import com.momentum4999.robot.subsystems.ClimberSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.LEDSubsystem;
import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.subsystems.TargetingSubsystem;
import com.momentum4999.robot.util.MoShuffleboard;

import org.usfirst.frc.team4999.controllers.LogitechF310;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Components
	public final MoInput input = new SingleControllerInput(new LogitechF310(0));
	public final PneumaticsControlModule pneumatics = new PneumaticsControlModule();
	public final PowerDistribution pdp = new PowerDistribution();

	// Joystick Buttons
	private final Button shoot = new Button(input::getRunShooter);

	// Subsystems
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final TargetingSubsystem targetingSubsystem = new TargetingSubsystem(driveSubsystem);
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(targetingSubsystem);
	public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	// Commands
	private final Command shooterIdleCommand = new ShooterIdleCommand(shooterSubsystem);
	private final Command shooterActiveNoTargetingCommand = new ShooterActiveCommand(shooterSubsystem, targetingSubsystem, false);
	private final Command shooterActiveWithTargetingCommand = new ShooterActiveCommand(shooterSubsystem, targetingSubsystem, true);

	private final DriveCommand driveCommand = new DriveCommand(this.driveSubsystem, input);
	private final Command climbCommand = new SequentialCommandGroup(
		new ZeroClimberCommand(climberSubsystem, pdp),
		new RunClimberCommand(climberSubsystem, input)
	);
	private final Command teleOpCommand = new ParallelCommandGroup(driveCommand, climbCommand);

	private final Command autoCommand = new SequentialCommandGroup(
		new ShooterActiveCommand(shooterSubsystem, targetingSubsystem, false).withTimeout(2.5),
		new AutoDriveCommand(driveSubsystem, 0.7, 0.7, 2)
	);

	// Shuffleboard
	public final MoShuffleboard shuffleboard = new MoShuffleboard();

	// LEDS
	public final LEDSubsystem leds = new LEDSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureDefaultCommands();
		configureButtonBindings();

		MoShuffleboard.matchTab()
			.add("Limelight", new HttpCamera("Limelight", "http://10.49.99.11:5800/", HttpCameraKind.kMJPGStreamer))
			.withSize(3, 3).withProperties(Map.of("Show controls", false));
	}

	private void configureDefaultCommands() {
		shooterSubsystem.setDefaultCommand(shooterIdleCommand);
	}

	/**
	 * Used to define button->command mappings.
	 */
	private void configureButtonBindings() {
		// ---------------------------------- Shooter ---------------------------------
		this.shoot.whenHeld(shooterActiveNoTargetingCommand);
	}

	/**
	 * Used to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return this.autoCommand;
	}

	public Command getRunningTeleopCommand() {
		return this.teleOpCommand;
	}

	public void teleopInit() {
		if (!this.targetingSubsystem.hasFirstInit()) {
			this.targetingSubsystem.resetTargetPose();
		}
	}

	public void robotPeriodic() {
		/*
		for (int i = 0; i < this.pdp.getNumChannels(); i++) {
			MoShuffleboard.putNumber("PDP Channel "+i, this.pdp.getCurrent(i));
		}*/
	}

	public void stopSubsystems() {
		this.driveSubsystem.stop();
	}
}
