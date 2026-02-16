// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperUptake;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Elastic;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
	// private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	// private double MaxAngularRate = 0 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	// /* Setting up bindings for necessary control of the swerve drive platform */
	// private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
	// .withDeadband(MaxSpeed * 0.1)
	// .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
	// .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	// private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	// private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	// private final Telemetry logger = new Telemetry(MaxSpeed);
	// public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final HopperUptake hopperUptake = new HopperUptake();
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
		// VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));
		// VersionConstants.logSignals();

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
	private void configureDriverControls() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		// drivetrain.setDefaultCommand(
		// // Drivetrain will execute this command periodically
		// drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
		// .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
		// .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
		// ));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		// final var idle = new SwerveRequest.Idle();
		// RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		driverController.a().whileTrue(hopperUptake.startBothCommand());
		driverController.b().whileTrue(hopperUptake.startUnJamCommand());
		driverController.x().whileTrue(hopperUptake.stopBothCommand());

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		// driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

		// drivetrain.registerTelemetry(logger::telemeterize);
	}

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
		// Simple drive forward auton
		// final var idle = new SwerveRequest.Idle();
		// return Commands.sequence(
		// // Reset our field centric heading to match the robot
		// // facing away from our alliance station wall (0 deg).
		// drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
		// // Then slowly drive forward (away from us) for 5 seconds.
		// drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
		// .withVelocityY(0)
		// .withRotationalRate(0))
		// .withTimeout(5.0),
		// // Finally idle for the rest of auton
		// drivetrain.applyRequest(() -> idle));
		return new Command() {

		};
	}
}
