package frc.robot.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.HopperUptake;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.superstructure.sources.ShootingSource;
import frc.robot.superstructure.sources.ShootingSourceIdle;
import frc.robot.util.AlertUtil;

import static edu.wpi.first.units.Units.Seconds;

/**
 * A superstructure that controls the functionality of the shooter. This helps coordinate the various subsystems involved with the shooter.
 * <p>
 * This class is built around the foundation of a state machine, and has a defined {@link ShooterState list of states} it can be in. This helps keep track of the different steps involved in shooting, which makes it a lot easier to tweak behavior and a lot harder to introduce weird bugs where things get into undefined states.
 * <p>
 * This class also has a concept of {@link ShootingSource shooting sources}. These provide target values for the shooter subsystems, and can be built to dynamically update, which aids in the development of things like {@link frc.robot.superstructure.sources.ShootFromAnywhereSource shoot-from-anywhere} functionality.
 */
@Logged
public class ShooterSuperstructure {
	@NotLogged
	private final Turret turret;
	@NotLogged
	private final Hood hood;
	@NotLogged
	private final Shooter flywheel;
	@NotLogged
	private final HopperUptake hopperUptake;
	@NotLogged
	private final DigitalInput uptakeBeamBreak;

	/**
	 * The list of states the shooter can be in.
	 */
	public enum ShooterState {
		/**
		 * Stopped. This won't automatically transition into any other states.
		 */
		OFF,
		/**
		 * Homed and waiting for a target.
		 * <p>
		 * If a target is given by the current source, this will automatically transition to {@link #SHOOTING}.
		 */
		HOME,
		/**
		 * Currently under manual control.
		 */
		MANUAL_CONTROL,
		/**
		 * The shooter has a target and is in some stage of shooting. We never transition directly to this state, only substates of it.
		 */
		SHOOTING,
		/**
		 * Has target, waiting for things to ready. Substate of {@link #SHOOTING}.
		 * <p>
		 * Once the subsystems are all ready, this will automatically transition to {@link #SHOOTING_STAGE_2_READY_TO_SHOOT}.
		 */
		SHOOTING_STAGE_1_INITIALIZING,
		/**
		 * Initialized and ready to shoot. Waiting for {@link ShooterTrigger#START_SHOOTING}.
		 */
		SHOOTING_STAGE_2_READY_TO_SHOOT,
		/**
		 * Various stages related to active shooting (at this point, control is completely automatic).
		 * <p>
		 * Once the hopper is out of balls (defined as when the hopper beam break hasn't detected a ball for {@link ShootingConstants#SHOOTING_TIMEOUT}), this will automatically transition back to {@link #HOME} state.
		 */
		SHOOTING_STAGE_3_SHOOTING,
		/**
		 * Waiting for the hood and uptake to ready, since we only spin those while actively shooting. This is a substate of {@link #SHOOTING_STAGE_3_SHOOTING}.
		 */
		SHOOTING_WAITING,
		/**
		 * Currently tracking and shooting at a target. Spindexer is spindexing.
		 */
		SHOOTING_RUNNING,
		/**
		 * We were {@link #SHOOTING_RUNNING shooting}, but for whatever reason we had to stop to wait on a mechanism to ready (maybe the turret reached a limit and had to wrap around).
		 * <p>
		 * Once we're ready again, we'll immediately transition back to {@link #SHOOTING_RUNNING}.
		 */
		SHOOTING_PAUSED,
	}

	/**
	 * Triggers for the shooter's state machine.
	 */
	enum ShooterTrigger {
		/**
		 * Turns off the shooter.
		 */
		TURN_OFF,
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
		 * Signals to the superstructure that we are ready to shoot.
		 */
		READY_TO_SHOOT,
		/**
		 * Signals to the shooter to start shooting.
		 */
		START_SHOOTING,
		/**
		 * Signals to the shooter to pause until mechanisms are ready.
		 */
		PAUSE_SHOOTING,
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
	 * Timer used to time the stopping of the shooter.
	 */
	private final Timer shooterTimeout = new Timer();

	/**
	 * The current source to poll for shooting values.
	 */
	private ShootingSource shootingSource = new ShootingSourceIdle();

	/**
	 * Creates a new ShooterSuperstructure.
	 *
	 * @param shooter
	 *            The shooter to control.
	 * @param hood
	 *            The hood to control.
	 * @param turret
	 *            The turret to control.
	 * @param hopperUptake
	 *            The hopper/uptake subsystem to control.
	 * @param uptakeBeamBreak
	 *            The beam break for the uptake.
	 */
	public ShooterSuperstructure(Shooter shooter, Hood hood, Turret turret, HopperUptake hopperUptake, DigitalInput uptakeBeamBreak) {
		this.flywheel = shooter;
		this.hood = hood;
		this.turret = turret;
		this.hopperUptake = hopperUptake;
		this.uptakeBeamBreak = uptakeBeamBreak;

		// Set up the state machine
		// ---- Off state ----
		stateMachineConfig.configure(ShooterState.OFF)
				.onEntry(this::stop)
				.permitReentry(ShooterTrigger.TURN_OFF)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		// ---- Home state ----
		stateMachineConfig.configure(ShooterState.HOME)
				.onEntry(this::home)
				.permitReentry(ShooterTrigger.HOME)
				.permit(ShooterTrigger.TURN_OFF, ShooterState.OFF)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.INITIALIZE, ShooterState.SHOOTING_STAGE_1_INITIALIZING);

		// ---- Manual control ----
		stateMachineConfig.configure(ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.TURN_OFF, ShooterState.OFF)
				.permit(ShooterTrigger.HOME, ShooterState.HOME);

		// ---- Shooting state -----
		stateMachineConfig.configure(ShooterState.SHOOTING)
				.permit(ShooterTrigger.TURN_OFF, ShooterState.OFF)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_1_INITIALIZING)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.READY_TO_SHOOT, ShooterState.SHOOTING_STAGE_2_READY_TO_SHOOT);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_2_READY_TO_SHOOT)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING_WAITING);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_3_SHOOTING)
				.substateOf(ShooterState.SHOOTING)
				// Spin up uptake while shooting
				.onEntry(hopperUptake::startUptakeForward)
				.onExit(hopperUptake::stopUptake);

		stateMachineConfig.configure(ShooterState.SHOOTING_RUNNING)
				.substateOf(ShooterState.SHOOTING_STAGE_3_SHOOTING)
				.permit(ShooterTrigger.PAUSE_SHOOTING, ShooterState.SHOOTING_PAUSED)
				// Run hopper while shooting
				.onEntry(hopperUptake::startHopperForward)
				.onExit(hopperUptake::stopHopper)
				// Restart the shooting timer when we start shooting
				.onEntry(shooterTimeout::restart)
				// And stop it when we stop
				.onExit(() -> {
					shooterTimeout.stop();
					shooterTimeout.reset();
				});

		stateMachineConfig.configure(ShooterState.SHOOTING_WAITING)
				.substateOf(ShooterState.SHOOTING_STAGE_3_SHOOTING)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING_RUNNING);

		stateMachineConfig.configure(ShooterState.SHOOTING_PAUSED)
				.substateOf(ShooterState.SHOOTING_STAGE_3_SHOOTING)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING_RUNNING);

		stateMachine = new StateMachine<>(ShooterState.OFF, stateMachineConfig);

		stateMachine.onUnhandledTrigger((ShooterState state, ShooterTrigger trigger) -> {
			AlertUtil.sendNotification(AlertUtil.AlertLevel.ERROR, "Unhandled trigger", "Unhandled trigger: " + trigger + " from state: " + state, Seconds.of(3.0));
		});

		// Put commands on SmartDashboard
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Turn Off", turnOffCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Home", homeCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Start Manual Control", startManualControlCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Start Shooting", startShootingCommand());
	}

	/**
	 * Updates the shooter state depending on the current shooting source.
	 * <p>
	 * The main purpose of this method is to handle transitions between different state machine states, as well as updating subsystem targets. Every time this method is run, it will check the current state of the superstructure, and, depending on the current state of the superstructure, the current value of the selected {@link ShootingSource}, and feedback from subsystem sensors, this will update the subsystem targets and transition to different states.
	 */
	public void update() {
		Optional<ShooterValues> targetShooterValues = shootingSource.get();

		// Check the current state and handle sequence transitions.
		switch (stateMachine.getState()) {
			case OFF:
				break;

			case HOME:
				// Homed and waiting for a target
				// This is just the idle state of the shooter

				// If we get a target shooter value, start to track the target
				if (targetShooterValues.isPresent()) {
					stateMachine.fire(ShooterTrigger.INITIALIZE);
				}
				break;

			case MANUAL_CONTROL:
				break;

			case SHOOTING:
				// We don't really have to worry about the shooting state itself since we are always in substates of it.
				break;

			case SHOOTING_STAGE_1_INITIALIZING:
				// Preparing to track a target

				// Get the shooter to an initial setpoint so we can start tracking
				if (targetShooterValues.isPresent()) {
					setValues(targetShooterValues.get(), false);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}

				// Once we reach the target, start to track
				if (subsystemsAtTargets()) {
					stateMachine.fire(ShooterTrigger.READY_TO_SHOOT);
					break;
				}

				break;

			case SHOOTING_STAGE_2_READY_TO_SHOOT:
				// Ready to shoot at a target, waiting for a command from drivers or auto to start shooting

				if (targetShooterValues.isPresent()) {
					// Keep tracking the target if we still have a target
					setValues(targetShooterValues.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;

			case SHOOTING_STAGE_3_SHOOTING:
				// We don't really have to worry about the shooting substate itself since we are always in substates of it.
				break;

			case SHOOTING_RUNNING:
				// Tracking the target and shooting

				// Shooter timeout
				if (!uptakeBeamBreak.get()) {
					shooterTimeout.restart();
				} else {
					// After a little while of the beam break being off, stop the shooter
					// if (shooterTimeout.hasElapsed(ShootingConstants.SHOOTING_TIMEOUT)) {
					// stateMachine.fire(ShooterTrigger.HOME);
					//
					// break;
					// }
				}

				if (subsystemsNeedPause()) {
					// If subsystems stray away from their targets (like if the turret has to wrap), pause shooting
					stateMachine.fire(ShooterTrigger.PAUSE_SHOOTING);
					break;
				}

				if (targetShooterValues.isPresent()) {
					// Keep tracking the target if we still have a target
					setValues(targetShooterValues.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;

			case SHOOTING_WAITING:
				// Was shooting, but needed to pause for subsystems to catch up

				if (targetShooterValues.isPresent()) {
					// Keep tracking the target if we still have a target
					setValues(targetShooterValues.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}

				if (subsystemsAtTargets()) {
					// Ready to start shooting again, so let's do that
					stateMachine.fire(ShooterTrigger.START_SHOOTING);
					break;
				}

				break;

			case SHOOTING_PAUSED:
				// Was shooting, but needed to pause for subsystems to catch up

				if (!subsystemsNeedPause()) {
					// Ready to start shooting again, so let's do that
					stateMachine.fire(ShooterTrigger.START_SHOOTING);
					break;
				}

				if (targetShooterValues.isPresent()) {
					// Keep tracking the target if we still have a target
					setValues(targetShooterValues.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;
		}
	}

	/**
	 * Command to turn off the shooter. This can be called from any state, and completely stops the shooter.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command turnOffCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.TURN_OFF))
				.ignoringDisable(true);
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
	 * Command to start shooting. This can only be called from the {@link ShooterState#SHOOTING_STAGE_2_READY_TO_SHOOT} state.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command startShootingCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.START_SHOOTING));
	}

	/**
	 * Command to start shooting. This will wait until we enter the {@link ShooterState#SHOOTING_STAGE_2_READY_TO_SHOOT} state to start shooting.
	 * <p>
	 * Exits when exiting the {@link ShooterState#SHOOTING_STAGE_3_SHOOTING} state.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command startShootingWhenReadyCommand() {
		return Commands.waitUntil(readyToShootTrigger())
				.andThen(startShootingCommand());
	}

	/**
	 * Sets the shooter state to the specified state. This requires that you are in {@link ShooterState#MANUAL_CONTROL}.
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
				AlertUtil.sendNotification(AlertUtil.AlertLevel.ERROR, "Not in manual mode", "Attempted to execute setStateCommand without being in manual mode.", Seconds.of(3.0));
			}
		}, flywheel, hood, turret);
	}

	/**
	 * Sets the shooting source to the given source. This can be called from any state.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @param source
	 *            The source to set.
	 * @return
	 *         Command to run.
	 */
	public Command setSourceCommand(ShootingSource source) {
		return Commands.runOnce(() -> setSource(source))
				.ignoringDisable(true);
	}

	/**
	 * Trigger that returns true when ready to shoot. Should be useful for triggering controller haptics.
	 *
	 * @return
	 *         Trigger, true when in {@link ShooterState#SHOOTING_STAGE_2_READY_TO_SHOOT}, false otherwise.
	 */
	public Trigger readyToShootTrigger() {
		return new Trigger(() -> stateMachine.isInState(ShooterState.SHOOTING_STAGE_2_READY_TO_SHOOT));
	}

	/**
	 * Checks if the subsystems are at their targets. This checks flywheel speed, hood position, turret position, and uptake speed.
	 *
	 * @return
	 *         Are all the shooter subsystems at their targets?
	 */
	public boolean subsystemsAtTargets() {
		return flywheel.isAtTarget() && hood.isAtPosition() && turret.isAtTarget() && hopperUptake.isUptakeAtTarget();
	}

	/**
	 * Checks if the subsystems need for shooting to pause. This checks if the turret is no longer at its target (we can assume other subsystems are close enough).
	 *
	 * @return
	 *         True if we need to pause shooting to wait for subsystems to catch up, false otherwise.
	 * @apiNote
	 *          This is intended for internal use, but is made public for logging purposes.
	 */
	public boolean subsystemsNeedPause() {
		// return !turret.isAtTarget();
		return false;
	}

	/**
	 * Gets the current state of the shooter state machine.
	 */
	public ShooterState getState() {
		return stateMachine.getState();
	}

	/**
	 * Sets the shooting source to use.
	 * <p>
	 * This can be called at any time, and will immediately take over the current setting.
	 *
	 * @param source
	 *            Source to set.
	 */
	public void setSource(ShootingSource source) {
		shootingSource = source;
	}

	/**
	 * Gets the name of the current shooting source.
	 */
	public String getSourceName() {
		return shootingSource.getName();
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
		flywheel.startFlywheel(values.getFlywheelSpeed());
		turret.setPosition(values.getTurretAngle());

		// This is a bit of a jank way to do this but I'll do something better later if I have time
		if (stateMachine.isInState(ShooterState.SHOOTING_STAGE_3_SHOOTING)) {
			hood.moveToPosition(values.getHoodAngle());
		} else {
			hood.moveToPosition(HoodConstants.HOME_ANGLE);
		}
	}

	/**
	 * Homes the various components of the turret.
	 */
	private void home() {
		flywheel.startFlywheel(ShooterConstants.IDLE_SPEED);
		hood.moveToPosition(HoodConstants.HOME_ANGLE);
		turret.unspool();
	}

	/**
	 * Stops the various components of the turret.
	 */
	private void stop() {
		flywheel.stopFlywheel();
		hood.moveToPosition(HoodConstants.HOME_ANGLE);
		turret.unspool();
	}
}
