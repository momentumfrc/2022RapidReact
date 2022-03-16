// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import java.util.Map;

import com.momentum4999.robot.commands.AutonomousCommand;
import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.input.InputDevice;
import com.momentum4999.robot.input.MoMultiGamepad;
import com.momentum4999.robot.input.InputDevice.InputButton;
import com.momentum4999.robot.input.InputDevice.JoystickButtonHolder;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.IntakeSubsystem;
import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.subsystems.TargetingSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem.Mode;
import com.momentum4999.robot.util.Components;
import com.momentum4999.robot.util.MoCode;
import com.momentum4999.robot.util.MoShuffleboard;
import com.momentum4999.robot.widgets.AutoScriptChooser;

import org.usfirst.frc.team4999.controllers.LogitechF310;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
	public final InputDevice gamepad = createGamepad();
	public final PneumaticsControlModule pneumatics = new PneumaticsControlModule();
	public final PowerDistribution pdp = new PowerDistribution();

	// Joystick Buttons
	private final JoystickButtonHolder intakeFwd = gamepad.getJoystickButton(InputButton.LB);
	private final JoystickButtonHolder intakeRev = gamepad.getJoystickButton(InputButton.A);
	private final JoystickButtonHolder shoot = gamepad.getJoystickButton(InputButton.RB);

	// Subsystems
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final TargetingSubsystem targetingSubsystem = new TargetingSubsystem(driveSubsystem);
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(targetingSubsystem);

	// Commands
	private final DriveCommand driveCommand = new DriveCommand(Mode.ARCADE, this.driveSubsystem, this.gamepad);
	private final AutonomousCommand autoCommand = new AutonomousCommand(this);

	// Shuffleboard
	public final MoShuffleboard shuffleboard = new MoShuffleboard();
	public final AutoScriptChooser autoScriptChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		MoCode.INSTANCE.loadScripts(this);

		configureButtonBindings();
		
		this.autoScriptChooser = new AutoScriptChooser(this); // Auto scripts need to be loaded before this is initialized
		MoShuffleboard.matchTab()
			.add("Limelight", new HttpCamera("Limelight", "http://10.49.99.11:5800/", HttpCameraKind.kMJPGStreamer))
			.withSize(3, 3).withProperties(Map.of("Show controls", false));
	}

	/**
	 * Used to define button->command mappings.
	 */
	private void configureButtonBindings() {
		// ---------------------------------- Intake ----------------------------------
		this.intakeFwd.apply(button -> {
			button.whenPressed(new InstantCommand(this::beginIntake, this.intakeSubsystem))
				.whenHeld(new RunCommand(() -> this.runIntake(false), this.intakeSubsystem))
				.whenReleased(new RunCommand(this::endIntake, this.intakeSubsystem));
		});
		this.intakeRev.apply(button -> {
			button.whenPressed(new InstantCommand(this::beginIntake, this.intakeSubsystem))
				.whenHeld(new RunCommand(() -> this.runIntake(true), this.intakeSubsystem))
				.whenReleased(new RunCommand(this::endIntake, this.intakeSubsystem));
		});
				
		// ---------------------------------- Shooter ---------------------------------
		this.shoot.apply(button -> {
			button.whenHeld(new RunCommand(this::shoot, this.shooterSubsystem))
				.whenReleased(new InstantCommand(this::stopShooting, this.shooterSubsystem));
		});
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

	public static InputDevice createGamepad() {
		return new MoMultiGamepad(
			MoMultiGamepad.entry(new LogitechF310(Components.LOGITECH_F310_PORT)),
			MoMultiGamepad.entry(new LogitechF310(Components.LOGITECH_F310_PORT + 1))
		);
		//return new MoSingleGamepad(new LogitechF310(Components.LOGITECH_F310_PORT));
	}

	public void beginIntake() {
		this.intakeSubsystem.setIntake(true);
	}

	public void endIntake() {
		this.intakeSubsystem.setIntake(false);
		this.intakeSubsystem.idleIntake();
		this.shooterSubsystem.idleIndexer();
	}

	public void runIntake(boolean rev) {
		// If forward, stop indexer when full
		// If reverse, spit out no matter what
		if (!this.shooterSubsystem.fullOfBalls() || rev) {
			this.shooterSubsystem.runIndexer(rev);
		} else {
			this.shooterSubsystem.idleIndexer();
		}

		this.intakeSubsystem.runIntake(rev);
	}

	public void shoot() {
		this.shooterSubsystem.runActive(true);
	}

	public void shootWithoutTargeting() {
		this.shooterSubsystem.runActive(false);
	}

	public void stopShooting() {
		this.shooterSubsystem.stop();
	}
}
