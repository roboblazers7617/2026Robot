// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainControls;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Elastic;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.MissingFormatWidthException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	public final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
			.withHeadingPID(6, 0, 0.1)
			.withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	public final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
			.withRotationalDeadband(DrivetrainConstants.MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // se open-loop control for drive motors
	private double MaxAngularRate = 0 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
	private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_SPEED);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final DrivetrainControls drivetrainControls = new DrivetrainControls(drivetrain);
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

	/* Path follower */
	// private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));
		VersionConstants.logSignals();

		// autoChooser = AutoBuilder.buildAutoChooser("Tests");
		// SmartDashboard.putData("Auto Mode", autoChooser);

		// Configure the trigger bindings
		configureNamedCommands();
		configureDriverControls();
		configureOperatorControls();

		// Warmup PathPlanner to avoid Java pauses
		FollowPathCommand.warmupCommand().schedule();
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
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> {
					if (Math.abs(driverController.getRightY()) >= 0.5 || Math.abs(driverController.getRightX()) >= 0.5) {
						drive.withTargetDirection(new Rotation2d(-driverController.getRightY(), -driverController.getRightX()));
					}

					return drive.withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED * drivetrainControls.speedMultiplier) // Drive forward with negative Y (forward)
							.withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED * drivetrainControls.speedMultiplier);// Drive left with negative X (left)
				}));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// start and stop
		driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
		// letter buttons
		driverController.a().whileTrue(drivetrain.applyRequest(() -> drivetrainControls.brake));
		driverController.b().whileTrue(drivetrain.applyRequest(() -> drivetrainControls.point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
		// bumpers
		driverController.leftBumper().onTrue(drivetrain.applyRequest(() -> drivetrainControls.spin.withRotationalRate(-driverController.getRightX() * MaxAngularRate)));
		driverController.rightBumper().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.SLOW_SPEED_MULTIPLIER));
		// triggers
		driverController.rightTrigger().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.MAX_SPEED_MULTIPLIER));

		drivetrain.registerTelemetry(logger::telemeterize);
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
		// // Simple drive forward auton
		// final var idle = new SwerveRequest.Idle();
		return Commands.sequence();
		// return autoChooser.getSelected();
		// // // Reset our field centric heading to match the robot
		// // // facing away from our alliance station wall (0 deg).
		// // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
		// // // Then slowly drive forward (away from us) for 5 seconds.
		// // drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
		// // .withVelocityY(0)
		// // .withRotationalRate(0))
		// // .withTimeout(5.0),
		// // // Finally idle for the rest of auton
		// // drivetrain.applyRequest(() -> idle));
	}
}
