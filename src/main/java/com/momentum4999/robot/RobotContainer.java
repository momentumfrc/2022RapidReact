// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.commands.ExampleCommand;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.MoSingleGamepad;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;
import com.momentum4999.robot.util.Components;

import org.usfirst.frc.team4999.controllers.LogitechF310;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

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
	private final InputDevice inputDevice = createInputDevice();

	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();

	// Commands
	private final DriveCommand driveCommand = new DriveCommand(Mode.ARCADE, this.driveSubsystem, this.inputDevice);
	private final ExampleCommand autoCommand = new ExampleCommand();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return this.autoCommand;
	}

	public Command getCurrentTeleopCommand() {
		// TODO: Change this
		return this.driveCommand;
	}

	public static InputDevice createInputDevice() {
		return new MoSingleGamepad(new LogitechF310(Components.LOGITECH_F310_PORT));
	}
}
