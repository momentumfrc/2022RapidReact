// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import java.util.Map;

import com.momentum4999.robot.commands.AutonomousCommand;
import com.momentum4999.robot.commands.RunClimberCommand;
import com.momentum4999.robot.commands.ShooterActiveCommand;
import com.momentum4999.robot.commands.ShooterIdleCommand;
import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.commands.ZeroClimberCommand;
import com.momentum4999.robot.input.MoBaseInput;
import com.momentum4999.robot.input.MoMultiInput;
import com.momentum4999.robot.input.InputMapping.InputButton;
import com.momentum4999.robot.input.MoBaseInput.JoystickButtonHolder;
import com.momentum4999.robot.subsystems.ClimberSubsystem;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.IntakeSubsystem;
import com.momentum4999.robot.subsystems.LEDSubsystem;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
	public final MoBaseInput gamepad = createGamepad();
	public final PneumaticsControlModule pneumatics = new PneumaticsControlModule();
	public final PowerDistribution pdp = new PowerDistribution();

	// Joystick Buttons
	private final JoystickButtonHolder intakeFwd = gamepad.getJoystickButton(InputButton.LB);
	private final JoystickButtonHolder intakeRev = gamepad.getJoystickButton(InputButton.A);
	private final JoystickButtonHolder shoot = gamepad.getJoystickButton(InputButton.RB);
	private final JoystickButtonHolder shootNoTarget = gamepad.getJoystickButton(InputButton.B);

	// Subsystems
	public final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final TargetingSubsystem targetingSubsystem = new TargetingSubsystem(driveSubsystem);
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(targetingSubsystem);
	public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(pdp);

	// Commands
	private final Command shooterIdleCommand = new ShooterIdleCommand(shooterSubsystem);
	private final Command shooterActiveNoTargetingCommand = new ShooterActiveCommand(shooterSubsystem, targetingSubsystem, false);
	private final Command shooterActiveWithTargetingCommand = new ShooterActiveCommand(shooterSubsystem, targetingSubsystem, true);

	private final DriveCommand driveCommand = new DriveCommand(Mode.ARCADE, this.driveSubsystem, this.gamepad);
	private final Command intakeCommand = new SequentialCommandGroup(
		new ZeroClimberCommand(climberSubsystem, pdp),
		new RunClimberCommand(climberSubsystem, this.gamepad)
	);
	private final Command teleOpCommand = new ParallelCommandGroup(driveCommand, intakeCommand);
	private final AutonomousCommand autoCommand = new AutonomousCommand(this);

	// Shuffleboard
	public final MoShuffleboard shuffleboard = new MoShuffleboard();
	public final AutoScriptChooser autoScriptChooser;

	// LEDS
	public final LEDSubsystem leds = new LEDSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		MoCode.INSTANCE.loadScripts(this);

		configureDefaultCommands();
		configureButtonBindings();

		this.autoScriptChooser = new AutoScriptChooser(this); // Auto scripts need to be loaded before this is initialized
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
		this.shoot.apply(button -> button.whenHeld(shooterActiveWithTargetingCommand));
		this.shootNoTarget.apply(button -> button.whenHeld(shooterActiveNoTargetingCommand));
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

	public static MoBaseInput createGamepad() {
		return new MoMultiInput(
			MoMultiInput.entry(new LogitechF310(Components.LOGITECH_F310_PORT)),
			MoMultiInput.entry(new LogitechF310(Components.LOGITECH_F310_PORT + 1))
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
		this.shooterSubsystem.idleShooter();
	}

	public void runIntake(boolean rev) {
		this.shooterSubsystem.runIndexer(rev);
		if (rev) {
			this.shooterSubsystem.retractShooter();
		}

		this.intakeSubsystem.runIntake(rev);
	}
}
