// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import com.momentum4999.robot.commands.AutonomousCommand;
import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.MoSingleGamepad;
import com.momentum4999.robot.input.InputDevice.InputButton;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.IntakeSubsystem;
import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;
import com.momentum4999.robot.util.Components;

import org.usfirst.frc.team4999.controllers.LogitechF310;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
	private final InputDevice gamepad = createGamepad();

	// Joystick Buttons
	private final JoystickButton intakeFwd = gamepad.getJoystickButton(InputButton.LB);
	private final JoystickButton intakeRev = gamepad.getJoystickButton(InputButton.A);
	private final JoystickButton intakeToggle = gamepad.getJoystickButton(InputButton.B);
	private final JoystickButton shoot = gamepad.getJoystickButton(InputButton.RB);

	// Subsystems
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	// Commands
	private final DriveCommand driveCommand = new DriveCommand(Mode.ARCADE, this.driveSubsystem, this.gamepad);
	private final AutonomousCommand autoCommand = new AutonomousCommand(this, AutonomousCommand.readAutoMoCodeScript());

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		configureButtonBindings();
	}

	/**
	 * Used to define button->command mappings.
	 */
	private void configureButtonBindings() {
		// ---------------------------------- Intake ----------------------------------
		this.intakeFwd.whenHeld(new RunCommand(() -> this.runIntake(false), this.intakeSubsystem, this.shooterSubsystem));
		this.intakeRev.whenHeld(new RunCommand(() -> this.runIntake(true), this.intakeSubsystem, this.shooterSubsystem));
		this.intakeToggle.whenPressed(new RunCommand(this.intakeSubsystem::toggleIntakeExtension, this.intakeSubsystem));
		this.shoot.whenHeld(new RunCommand(() -> this.shooterSubsystem.shootAndIndex(), this.shooterSubsystem))
				.whenReleased(new RunCommand(() -> this.shooterSubsystem.idle(), this.shooterSubsystem));
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
		return this.driveCommand;
	}

	public void stopSubsystems() {
		this.driveSubsystem.stop();
	}

	public void runIntake(boolean rev) {
		this.intakeSubsystem.runIntake(rev);
		this.shooterSubsystem.indexWhileIntaking(rev);
	}

	public static InputDevice createGamepad() {
		/* USE FOR TWO DRIVER MODE
		return new MoMultiGamepad(
			MoMultiGamepad.entry(new LogitechF310(Components.LOGITECH_F310_PORT)),
			MoMultiGamepad.entry(new LogitechF310(Components.LOGITECH_F310_PORT + 1))
		);
		*/
		return new MoSingleGamepad(new LogitechF310(Components.LOGITECH_F310_PORT));
	}
}
