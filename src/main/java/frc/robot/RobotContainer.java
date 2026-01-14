// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Elastic;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
	/**
	 * The Controller used by the Driver of the robot, primarily controlling the drivetrain.
	 */
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	/**
	 * The Controller used by the Operator of the robot, primarily controlling the superstructure.
	 */
	@NotLogged
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));
		VersionConstants.logSignals();

		// Configure the trigger bindings
		configureNamedCommands();
		configureDriverControls();
		configureOperatorControls();
	}

	/**
	 * This method is run at the start of Auto.
	 */
	public void autoInit() {
		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.AUTO_TAB_NAME);
		}
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.TELEOP_TAB_NAME);
		}
	}

	/**
	 * Sets up the {@link NamedCommands} used by the autonomous routine.
	 */
	private void configureNamedCommands() {}

	/**
	 * Configures {@link Trigger Triggers} to bind Commands to the Driver Controller buttons.
	 */
	private void configureDriverControls() {}

	/**
	 * Configures {@link Triggers} to bind Commands to the Operator Controller buttons.
	 */
	private void configureOperatorControls() {}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return Commands.none();
	}
}
