// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.momentum4999.robot;

import com.momentum4999.robot.commands.DriveCommand;
import com.momentum4999.robot.commands.ShooterActiveCommand;
import com.momentum4999.robot.commands.ShooterIdleCommand;
import com.momentum4999.robot.input.MoInput;
import com.momentum4999.robot.input.SingleControllerInput;
import com.momentum4999.robot.subsystems.DriveSubsystem;
import com.momentum4999.robot.subsystems.HornSubsystem;
import com.momentum4999.robot.subsystems.LEDSubsystem;
import com.momentum4999.robot.subsystems.ShooterSubsystem;
import com.momentum4999.robot.util.MoShuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;

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
    private final Trigger honk = new Trigger(input::getHonkHorn);

    // Subsystems
    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final HornSubsystem hornSubsystem = new HornSubsystem();

    // Commands
    private final Command shooterIdleCommand = new ShooterIdleCommand(shooterSubsystem);
    private final Command shooterActiveNoTargetingCommand = new ShooterActiveCommand(shooterSubsystem);

    private final DriveCommand driveCommand = new DriveCommand(this.driveSubsystem, input);
    private final Command teleOpCommand = driveCommand;

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
    }

    private void configureDefaultCommands() {
        shooterSubsystem.setDefaultCommand(shooterIdleCommand);

        hornSubsystem.setDefaultCommand(new RunCommand(hornSubsystem::unhonk, hornSubsystem));
    }

    /**
     * Used to define button->command mappings.
     */
    private void configureButtonBindings() {
        // ---------------------------------- Shooter ---------------------------------
        this.shoot.whileTrue(shooterActiveNoTargetingCommand);
        honk.whileTrue(new RunCommand(hornSubsystem::honk, hornSubsystem));
    }

    /**
     * Used to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.print("No auto here!");
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
