package frc.robot.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StubbedHood;
import frc.robot.subsystems.StubbedHopperUptake;
import frc.robot.subsystems.StubbedShooter;
import frc.robot.subsystems.StubbedTurret;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;

/**
 * A superstructure that controls the functionality of the shooter. This helps coordinate the various subsystems involved with the shooter.
 */
@Logged
public class ShooterSuperstructure {
	// TODO: Implement things here for the various subsystems once those are added
	@NotLogged
	private final CommandSwerveDrivetrain drivetrain;
	@NotLogged
	private final StubbedTurret turret;
	@NotLogged
	private final StubbedHood hood;
	@NotLogged
	private final StubbedShooter flywheel;
	@NotLogged
	private final StubbedHopperUptake hopperUptake;

	/**
	 * The list of states the shooter can be in.
	 */
	enum ShooterState {
		/**
		 * Homed and waiting for a target.
		 */
		HOME,
		/**
		 * Currently under manual control.
		 */
		MANUAL_CONTROL,
		/**
		 * Has target, waiting for things to ready.
		 */
		INITIALIZING,
		/**
		 * Initialized and currently tracking and shooting at a target.
		 */
		SHOOTING,
	}

	/**
	 * Triggers for the shooter's state machine.
	 */
	enum ShooterTrigger {
		/**
		 * Home the shooter.
		 */
		HOME,
		/**
		 * Start manual control. This can be done from any state. Exiting manual control is done by homing.
		 */
		START_MANUAL_CONTROL,
		/**
		 * Starts initializing the shooter. At this point the shooter has a target.
		 */
		INITIALIZE,
		/**
		 * Signals to the shooter that initialization is done.
		 */
		INITIALIZING_DONE,
	}

	/**
	 * The configuration for the state machine.
	 */
	private final StateMachineConfig<ShooterState, ShooterTrigger> stateMachineConfig = new StateMachineConfig<>();
	/**
	 * The state machine.
	 */
	private final StateMachine<ShooterState, ShooterTrigger> stateMachine;

	/**
	 * Creates a new ShooterController.
	 *
	 * @param drivetrain
	 *            The drivetrain to poll the pose from.
	 * @param shooter
	 *            The shooter to control.
	 * @param hood
	 *            The hood to control.
	 * @param turret
	 *            The turret to control.
	 * @param hopperUptake
	 *            The hopper/uptake subsystem to control.
	 */
	public ShooterSuperstructure(CommandSwerveDrivetrain drivetrain, StubbedShooter shooter, StubbedHood hood, StubbedTurret turret, StubbedHopperUptake hopperUptake) {
		this.drivetrain = drivetrain;
		this.flywheel = shooter;
		this.hood = hood;
		this.turret = turret;
		this.hopperUptake = hopperUptake;

		// Set up the state machine
		stateMachineConfig.configure(ShooterState.HOME)
				.onEntry(this::home)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.INITIALIZE, ShooterState.INITIALIZING);

		stateMachineConfig.configure(ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.HOME, ShooterState.HOME);

		stateMachineConfig.configure(ShooterState.INITIALIZING)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.INITIALIZING_DONE, ShooterState.SHOOTING);

		stateMachineConfig.configure(ShooterState.SHOOTING)
				.onEntry(hopperUptake::start)
				.onExit(hopperUptake::stop)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		stateMachine = new StateMachine<>(ShooterState.HOME, stateMachineConfig);

		stateMachine.onUnhandledTrigger((ShooterState state, ShooterTrigger trigger) -> {
			DriverStation.reportError("Unhandled trigger: " + trigger + " from state: " + state, true);
		});

		// Put commands on SmartDashboard
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Home", homeCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Start Manual Control", startManualControlCommand());
	}

	/**
	 * Updates the shooter state depending on drivetrain position.
	 * <p>
	 * This method does quite a few things. Based off of the robot pose, this will
	 * find the desired shooting pose, and, depending on what state it's in, it
	 * will transition to other states as needed. Also, while shooting, this handles
	 * updating the tracking for the shooter.
	 */
	public void update() {
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(drivetrain.getState().Pose);

		// Check the current state and handle sequence transitions.
		switch (stateMachine.getState()) {
			case HOME:
				// Homed and waiting for a target
				// This is just the idle state of the shooter

				// If we get a target pose, start to track the target
				if (targetPose.isPresent()) {
					stateMachine.fire(ShooterTrigger.INITIALIZE);
				}
				break;

			case MANUAL_CONTROL:
				break;

			case INITIALIZING:
				// Preparing to track a target

				// Once we reach the target, start to track and shoot
				if (subsystemsAtTargets()) {
					stateMachine.fire(ShooterTrigger.INITIALIZING_DONE);
					break;
				}

				// Get the shooter to an initial setpoint with MotionMagic so we can start tracking
				if (targetPose.isPresent()) {
					prepareShootAtTarget(targetPose.get(), false);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;

			case SHOOTING:
				// Track the target without MotionMagic
				// We don't use MotionMagic here because it should improve tracking accuracy
				if (targetPose.isPresent()) {
					prepareShootAtTarget(targetPose.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;
		}
	}

	/**
	 * Command to home the shooter. This can be called from any state, and places the shooter back into automatic control.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command homeCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.HOME))
				.ignoringDisable(true);
	}

	/**
	 * Command to start manual control of the shooter. This can be called from any state, and allows for controlling the shooter with the other commands.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command startManualControlCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.START_MANUAL_CONTROL))
				.ignoringDisable(true);
	}

	/**
	 * Sets the shooter state to the specified state. This requires that you are in manual mode.
	 * <p>
	 * This assumes that we are not tracking a target, since that is done internally.
	 *
	 * @param values
	 *            Supplier for the values to set the shooter to.
	 * @return
	 *         Command to run.
	 */
	public Command setStateCommand(Supplier<ShooterValues> values) {
		return Commands.run(() -> {
			if (stateMachine.isInState(ShooterState.MANUAL_CONTROL)) {
				setValues(values.get(), false);
			} else {
				DriverStation.reportWarning("Attempted to execute setStateCommand without being in manual mode.", true);
			}
		}, flywheel, hood, turret);
	}

	/**
	 * Command to prepare to shoot at a certain target pose from the drivetrain's current pose. This requires that you are in manual mode.
	 * <p>
	 * This assumes that we are not tracking a target, since that is done internally.
	 *
	 * @param targetPose
	 *            Supplier for the pose to shoot at.
	 * @return
	 *         Command to run.
	 */
	public Command prepareShootAtTargetCommand(Supplier<Pose3d> targetPose) {
		return Commands.run(() -> {
			if (stateMachine.isInState(ShooterState.MANUAL_CONTROL)) {
				prepareShootAtTarget(targetPose.get(), false);
			} else {
				DriverStation.reportWarning("Attempted to execute prepareShootAtTargetCommand without being in manual mode.", true);
			}
		}, flywheel, hood, turret);
	}

	/**
	 * Checks if the subsystems are at their targets. This checks flywheel speed, hood position, and turret position.
	 *
	 * @return
	 *         Are all the shooter subsystems at their targets?
	 */
	public boolean subsystemsAtTargets() {
		return flywheel.isAtTarget() && hood.isAtTarget() && turret.isAtTarget();
	}

	/**
	 * Gets the current state of the shooter state machine.
	 */
	public ShooterState getState() {
		return stateMachine.getState();
	}

	/**
	 * Gets the target pose for the current drivetrain position.
	 * <p>
	 * This mostly just exists for logging.
	 *
	 * @return
	 *         The pose of the shooting target for the current drivetrain position.
	 */
	public Pose3d getTargetForPosition() {
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(drivetrain.getState().Pose);

		if (targetPose.isPresent()) {
			return targetPose.get();
		} else {
			return Pose3d.kZero;
		}
	}

	/**
	 * Prepares to shoot at a certain target pose from the drivetrain's current pose.
	 *
	 * @param targetPose
	 *            The pose to shoot at.
	 * @param tracking
	 *            Is the shooter currently tracking a target?
	 */
	private void prepareShootAtTarget(Pose3d targetPose, boolean tracking) {
		Pose2d robotPose = drivetrain.getState().Pose;

		setValues(ShootingCalculator.solve(new Pose3d(robotPose), targetPose), tracking);
	}

	/**
	 * Sets the shooter values to the specified values.
	 *
	 * @param values
	 *            The values to set the shooter to.
	 * @param tracking
	 *            Is the shooter currently tracking a target?
	 */
	private void setValues(ShooterValues values, boolean tracking) {
		// TODO: Update this once subsystems are in place
		flywheel.startShooter(values.getFlywheelSpeed(), !tracking);
		hood.setAngle(values.getHoodAngle(), !tracking);
		turret.setPosition(values.getTurretAngle(), !tracking);
	}

	/**
	 * Homes the various components of the turret.
	 */
	private void home() {
		// TODO: Update this once subsystems are in place
		flywheel.startShooter(RPM.of(10), true);
		hood.setAngle(Degrees.of(90), true);
		turret.unspool();

		// Temporary debug stuff
		System.out.println("Homed shooter");
	}
}
