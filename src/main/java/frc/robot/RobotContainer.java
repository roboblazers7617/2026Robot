// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainControls;
import frc.robot.subsystems.StubbedHood;
import frc.robot.subsystems.StubbedHopperUptake;
import frc.robot.subsystems.StubbedIntakeGrabber;
import frc.robot.subsystems.StubbedIntakeShoulder;
import frc.robot.subsystems.StubbedFlywheel;
import frc.robot.subsystems.StubbedTurret;
import frc.robot.superstructure.IntakeSuperstructure;
import frc.robot.superstructure.ShooterSuperstructure;
import frc.robot.superstructure.ShooterSuperstructureDebug;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Elastic;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	// swerve request for face heading on right stick
	public final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * 0.1)
			.withHeadingPID(5, 0, 0.1)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Add a 10% deadband
	private SendableChooser<Command> autoChooser;

	// swerve request for regular spinny (the defualt this year)
	public final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * 0.1)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // se open-loop control for drive motors

	// Define subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final DrivetrainControls drivetrainControls = new DrivetrainControls(drivetrain);
	public final StubbedFlywheel shooter = new StubbedFlywheel();
	public final StubbedHood hood = new StubbedHood();
	public final StubbedTurret turret = new StubbedTurret();
	public final StubbedHopperUptake hopperUptake = new StubbedHopperUptake();
	public final StubbedIntakeGrabber intakeGrabber = new StubbedIntakeGrabber();
	public final StubbedIntakeShoulder intakeShoulder = new StubbedIntakeShoulder();

	private final RebuiltDashboard rebuiltDashboard = new RebuiltDashboard(drivetrain, this);
	private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_SPEED_DEADBAND);

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
	 * Superstructure that handles controlling the shooter and related subsystems.
	 */
	private final ShooterSuperstructure shooterSuperstructure = new ShooterSuperstructure(drivetrain, shooter, hood, turret, hopperUptake);
	/**
	 * Debug controls for the ShooterController. Only initialized in {@link LoggingConstants#DEBUG_MODE debug mode}.
	 */
	private ShooterSuperstructureDebug shooterSuperstructureDebug;

	/**
	 * Superstructure that handles controlling the intake and related subsystems.
	 */
	private final IntakeSuperstructure intakeSuperstructure = new IntakeSuperstructure(intakeShoulder, intakeGrabber);

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
		if (LoggingConstants.DEBUG_MODE) {
			shooterSuperstructureDebug = new ShooterSuperstructureDebug(shooterSuperstructure);
		}
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
	 * This method handles the periodic functionality for the superstructure.
	 * This is done seperate from the subsystem periodic so it can be updated more frequently.
	 */
	public void superstructurePeriodic() {
		shooterSuperstructure.update();
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
					return spin.withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED_SWERVE * drivetrainControls.speedMultiplier) // Drive forward with negative Y (forward)
							.withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED_SWERVE * drivetrainControls.speedMultiplier)
							.withRotationalRate(-driverController.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND);// Drive left with negative X (left)
				}));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// start and stop
		driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
		// letter buttons
		driverController.a().whileTrue(drivetrain.applyRequest(() -> drivetrainControls.brake));
		// bumpers
		driverController.leftBumper().whileTrue(Commands.runOnce(() -> drive.withTargetDirection(drivetrain.getState().Pose.getRotation())).andThen(drivetrain.applyRequest(() -> {
			if (Math.abs(driverController.getRightY()) >= 0.5 || Math.abs(driverController.getRightX()) >= 0.5) {
				drive.withTargetDirection(new Rotation2d(-driverController.getRightY(), -driverController.getRightX()));
			}

			return drive.withVelocityX(-driverController.getLeftY() * DrivetrainConstants.MAX_SPEED_SWERVE * drivetrainControls.speedMultiplier) // Drive forward with negative Y (forward)
					.withVelocityY(-driverController.getLeftX() * DrivetrainConstants.MAX_SPEED_SWERVE * drivetrainControls.speedMultiplier);// Drive left with negative X (left)
		})));
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
		// TODO: I removed the simple auton and use the selected one instead, do we still need this.
		return autoChooser.getSelected();
		// // Simple drive forward auton
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
	}

	public void setAutoChooser(SendableChooser<Command> auto) {
		autoChooser = auto;
	}

	/**
	 * Gets the current value of the uptake beam break.
	 *
	 * @return
	 *         True if there is a ball in uptake, false otherwise.
	 */
	public boolean getIsBallInUptake() {
		return true;
	}
}
