// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import java.util.Map;

import com.momentum4999.robot.commands.AutoDriveCommand;
import com.momentum4999.robot.commands.ShooterActiveCommand;
import com.momentum4999.robot.commands.ShooterIdleCommand;
import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.input.SingleControllerInput;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.LEDSubsystem;
import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.util.MoShuffleboard;


import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
	public final MoInput input = new SingleControllerInput(new XboxController(0));

	// Joystick Buttons
	private final Trigger shoot = new Trigger(input::getRunShooter);

	// Subsystems
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	// Commands
	private final Command shooterIdleCommand = new ShooterIdleCommand(shooterSubsystem);
	private final Command shooterActiveNoTargetingCommand = new ShooterActiveCommand(shooterSubsystem);

	private final DriveCommand driveCommand = new DriveCommand(this.driveSubsystem, input);
	private final Command teleOpCommand = driveCommand;

	private final Command autoCommand = new SequentialCommandGroup(
		new ShooterActiveCommand(shooterSubsystem).withTimeout(2.5),
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
		this.shoot.whileTrue(shooterActiveNoTargetingCommand);
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

	public void teleopInit() {}

	public void robotPeriodic() {}

	public void stopSubsystems() {
		this.driveSubsystem.stop();
	}
}
