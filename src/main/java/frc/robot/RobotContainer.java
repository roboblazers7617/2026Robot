// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainControls;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.HopperUptake;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.intake.IntakeGrabber;
import frc.robot.subsystems.intake.IntakeShoulder;
import frc.robot.superstructure.ShooterSim;
import frc.robot.superstructure.ShooterSuperstructure;
import frc.robot.superstructure.ShooterSuperstructureDebug;
import frc.robot.superstructure.sources.ShootingSource;
import frc.robot.superstructure.sources.ShootFromAnywhereInterpolatedSource;
import frc.robot.superstructure.sources.ShootingSourceConstant;
import frc.robot.superstructure.sources.ShootingSourceIdle;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.HapticCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.util.Util;
import frc.robot.util.Elastic;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Seconds;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
	private final NetworkTable networkTableInst = (NetworkTableInstance.getDefault().getTable("/RoboBlazers"));

	// swerve request for face heading on right stick
	public final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * 0.1)
			// TODO: (Caleb) Please put in constants file
			.withHeadingPID(5, 0, 0.1)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Add a 10% deadband
	private SendableChooser<Command> autoChooser;

	// swerve request for regular spinny (the defualt this year)
	public final SwerveRequest.FieldCentric spin = new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED_DEADBAND * 0.1)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE_DEADBAND * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // se open-loop control for drive motors

	/**
	 * The beam break on the uptake.
	 * <p>
	 * Reads true if there is a ball going through uptake, false otherwise.
	 */
	public final DigitalInput uptakeBeamBreak = new DigitalInput(HopperConstants.BEAM_BREAK_DIO_PIN);
	public final DIOSim uptakeBeamBreakSim = new DIOSim(uptakeBeamBreak);

	// Define subsystems
	private final Telemetry logger = new Telemetry(DrivetrainConstants.MAX_SPEED_DEADBAND);
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final DrivetrainControls drivetrainControls = new DrivetrainControls(drivetrain);
	private final Vision vision = new Vision(drivetrain);
	private final RebuiltDashboard rebuiltDashboard = new RebuiltDashboard(drivetrain, this, logger);
	private final Shooter shooter;
	private final Hood hood;
	private final Turret turret = new Turret();
	private final HopperUptake hopperUptake = new HopperUptake();
	private final IntakeGrabber intakeGrabber = new IntakeGrabber();
	private final IntakeShoulder intakeShoulder = new IntakeShoulder();

	/**
	 * The Controller used by the Driver of the robot, primarily controlling the
	 * drivetrain.
	 */
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	/**
	 * The Controller used by the Operator of the robot, primarily controlling the
	 * superstructure.
	 */
	@NotLogged
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

	/**
	 * Superstructure that handles controlling the shooter and related subsystems.
	 */
	private final ShooterSuperstructure shooterSuperstructure;
	/**
	 * Debug controls for the ShooterController. Only initialized in {@link LoggingConstants#DEBUG_MODE debug mode}.
	 */
	private ShooterSuperstructureDebug shooterSuperstructureDebug;
	/**
	 * Simulation functionality for the shooter. Only initialized when in simulation.
	 */
	private ShooterSim shooterSim;

	/**
	 * The source that we use for shoot-from-anywhere.
	 */
	private final ShootingSource shootFromAnywhereSource = new ShootFromAnywhereInterpolatedSource(drivetrain);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publish();

		// Publish some general metadata
		Util.recordMetadata("DebugMode", LoggingConstants.DEBUG_MODE ? "true" : "false");
		Util.recordMetadata("LogLevel", LoggingConstants.DEBUG_LEVEL.name());

		// autoChooser = AutoBuilder.buildAutoChooser("Tests");
		// SmartDashboard.putData("Auto Mode", autoChooser);
		//
		// Set up subsystems
		shooter = new Shooter(networkTableInst.getSubTable("Shooter"));
		hood = new Hood(networkTableInst.getSubTable("Hood"));

		// Set up motor temperature warnings for Drivetrain
		for (SwerveModule<TalonFX, TalonFX, CANcoder> module : drivetrain.getModules()) {
			MotorMonitor.addMotor(module.getDriveMotor());
			MotorMonitor.addMotor(module.getSteerMotor());
		}

		// Set up superstructure
		shooterSuperstructure = new ShooterSuperstructure(shooter, hood, turret, hopperUptake, uptakeBeamBreak);

		// Configure the trigger bindings
		configureNamedCommands();
		configureDriverControls();
		configureOperatorControls();

		if (LoggingConstants.DEBUG_MODE) {
			shooterSuperstructureDebug = new ShooterSuperstructureDebug(shooterSuperstructure);
		}

		// Set up a trigger to go into the home state when we enable in auto
		RobotModeTriggers.autonomous()
				.onTrue(shooterSuperstructure.homeCommand());

		if (SuperstructureConstants.HOME_ON_ENABLE) {
			// Set up a trigger to go into the home state when we enable in teleop
			RobotModeTriggers.teleop()
					.onTrue(shooterSuperstructure.homeCommand());
		}

		// Set up a trigger so, when we enable in teleop we go into shoot while move
		RobotModeTriggers.teleop()
				.onTrue(shooterSuperstructure.setSourceCommand(shootFromAnywhereSource));

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
	 * This function is called once when the robot is first started up whilst in simulation.
	 */
	public void simulationInit() {
		shooterSim = new ShooterSim(drivetrain, shooter, hood, turret, hopperUptake);
	}

	/**
	 * This function is called periodically whilst in simulation.
	 */
	public void simulationPeriodic() {
		shooterSim.simulationPeriodic();
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
	private void configureNamedCommands() {
		NamedCommands.registerCommand("Deploy Intake", intakeShoulder.lowerIntakeCommand()
				.andThen(intakeGrabber.startIntakeCommand()));
		NamedCommands.registerCommand("Stow Intake", intakeShoulder.raiseIntakeSlowCommand()
				.andThen(intakeGrabber.startIntakeSlowCommand())
				.andThen(Commands.waitUntil(intakeShoulder::getIsAtTarget))
				.andThen(intakeGrabber.stopIntakeCommand()));

		NamedCommands.registerCommand("Raise Climb", Commands.print("The Climb will rise"));
		NamedCommands.registerCommand("Lower Climb", Commands.print("The Climb will fall"));

		NamedCommands.registerCommand("Protect Intake", intakeShoulder.stowOverBumperCommand());

		NamedCommands.registerCommand("Shoot", shooterSuperstructure.setSourceCommand(shootFromAnywhereSource)
				.andThen(Commands.waitUntil(shooterSuperstructure.readyToShootTrigger()))
				.andThen(shooterSuperstructure.startShootingCommand())
				.andThen(intakeShoulder.stowOverBumperCommand())
				.andThen(Commands.waitSeconds(4.0))
				.andThen(shooterSuperstructure.homeCommand()));
	}

	/**
	 * Configures {@link Trigger Triggers} to bind Commands to the Driver Controller
	 * buttons.
	 */
	private void configureDriverControls() {
		// Drivetrain will execute this command periodically
		// // Applies the default field centric command
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drivetrainControls.fieldCentricRequest(driverController)));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// --- start and stop ---
		// // resets gyro
		driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

		// --- letter buttons ---
		// // moves swerves into x formation
		driverController.a().whileTrue(drivetrain.applyRequest(() -> drivetrainControls.brake));

		// --- bumpers ---
		// // switches swerve requests to field centric facing angle
		driverController.leftBumper().whileTrue(Commands.runOnce(() -> drivetrainControls.setPose2()).andThen(drivetrain.applyRequest(() -> drivetrainControls.feildCentricFacingAngleRequest(driverController))));
		// // Sets multiplier to the lower value
		driverController.rightBumper().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.SLOW_SPEED_MULTIPLIER));

		// --- triggers ---
		// // Sets multiplier to the higher value
		driverController.rightTrigger().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.MAX_SPEED_MULTIPLIER));

		driverController.leftTrigger()
				.whileTrue(hopperUptake.startUptakeUnjamCommand())
				.onFalse(hopperUptake.startUptakeForwardCommand());

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Operator Controller
	 * buttons.
	 */
	private void configureOperatorControls() {
		// Wait until ready to shoot, then shoot in current mode
		// Home on release
		operatorController.leftTrigger()
				.whileTrue(shooterSuperstructure.startShootingWhenReadyCommand())
				.onFalse(shooterSuperstructure.homeCommand());

		// Wait until ready to shoot, then shoot in current mode while pulling in intake
		// Home on release
		operatorController.leftBumper()
				.whileTrue(Commands.waitUntil(shooterSuperstructure.readyToShootTrigger())
						.andThen(intakeGrabber.startIntakeSlowCommand())
						.andThen(intakeShoulder.raiseIntakeCommand())
						.andThen(shooterSuperstructure.startShootingCommand()))
				.onFalse(shooterSuperstructure.homeCommand()
						.andThen(intakeGrabber.stopIntakeCommand()));

		operatorController.rightBumper()
				.onTrue(intakeShoulder.stowOverBumperCommand());

		// Set mode to shoot from fixed position, wait until ready to shoot, then shoot
		// Home and set back to shoot from anywhere on release
		operatorController.rightTrigger()
				.whileTrue(shooterSuperstructure.setSourceCommand(new ShootingSourceConstant("Static Shoot", ShootingConstants.STATIC_SHOOT_VALUES))
						.andThen(shooterSuperstructure.startShootingWhenReadyCommand()))
				.onFalse(shooterSuperstructure.setSourceCommand(shootFromAnywhereSource)
						.andThen(shooterSuperstructure.homeCommand()));

		operatorController.povLeft()
				.whileTrue(shooterSuperstructure.setSourceCommand(new ShootingSourceConstant("Static Shoot - Left", ShootingConstants.STATIC_SHOOT_LEFT_DOOR_VALUES))
						.andThen(shooterSuperstructure.startShootingWhenReadyCommand()))
				.onFalse(shooterSuperstructure.setSourceCommand(shootFromAnywhereSource)
						.andThen(shooterSuperstructure.homeCommand()));

		operatorController.povRight()
				.whileTrue(shooterSuperstructure.setSourceCommand(new ShootingSourceConstant("Static Shoot - Right", ShootingConstants.STATIC_SHOOT_RIGHT_DOOR_VALUES))
						.andThen(shooterSuperstructure.startShootingWhenReadyCommand()))
				.onFalse(shooterSuperstructure.setSourceCommand(shootFromAnywhereSource)
						.andThen(shooterSuperstructure.homeCommand()));

		operatorController.povUp()
				.whileTrue(shooterSuperstructure.setSourceCommand(new ShootingSourceConstant("Static Shoot - Center", ShootingConstants.STATIC_SHOOT_CENTER))
						.andThen(shooterSuperstructure.startShootingWhenReadyCommand()))
				.onFalse(shooterSuperstructure.setSourceCommand(shootFromAnywhereSource)
						.andThen(shooterSuperstructure.homeCommand()));

		// Set mode to idle
		operatorController.povDown()
				.onTrue(shooterSuperstructure.setSourceCommand(new ShootingSourceIdle()));

		operatorController.back()
				.onTrue(shooterSuperstructure.turnOffCommand());
		operatorController.start()
				.onTrue(shooterSuperstructure.homeCommand());

		// Press A to START and LOWER INTAKE
		operatorController.a()
				.onTrue(intakeGrabber.startIntakeCommand())
				.onTrue(intakeShoulder.lowerIntakeCommand());
		// Press B to STOP INTAKE
		operatorController.b()
				.onTrue(intakeGrabber.stopIntakeCommand());
		// Press X to START INTAKE SLOWLY
		operatorController.x()
				.onTrue(intakeGrabber.startIntakeSlowCommand());
		// Press Y to STOW and then STOP INTAKE
		operatorController.y()
				.onTrue(intakeShoulder.raiseIntakeSlowCommand()
						.andThen(intakeGrabber.startIntakeSlowCommand())
						.andThen(Commands.waitUntil(intakeShoulder::getIsAtTarget))
						.finallyDo(intakeGrabber::stopIntake));

		// Manual control on joystick
		intakeShoulder.setDefaultCommand(intakeShoulder.nudgeIntakeCommand(() -> {
			double controllerValue = -operatorController.getRightY();

			if (Math.abs(controllerValue) > OperatorConstants.DEADBAND) {
				return controllerValue;
			} else {
				return 0.0;
			}
		}));

		// Haptics when ready to shoot
		shooterSuperstructure.readyToShootTrigger()
				.onTrue(new HapticCommand(operatorController, RumbleType.kLeftRumble, 0.5, Seconds.of(0.25))
						.onlyIf(RobotModeTriggers.teleop()));
	}

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
		return uptakeBeamBreak.get();
	}
}
