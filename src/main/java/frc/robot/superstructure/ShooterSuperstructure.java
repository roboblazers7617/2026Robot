package frc.robot.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StubbedHood;
import frc.robot.subsystems.StubbedHopperUptake;
import frc.robot.subsystems.StubbedFlywheel;
import frc.robot.subsystems.StubbedTurret;
import frc.robot.util.AlertUtil;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

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
	private final StubbedFlywheel flywheel;
	@NotLogged
	private final StubbedHopperUptake hopperUptake;
	@NotLogged
	private final DigitalInput uptakeBeamBreak;

	/**
	 * The list of states the shooter can be in.
	 */
	enum ShooterState {
		/**
		 * Homed and waiting for a target.
		 * <p>
		 * If a target is found for our current position, this will automatically transition to {@link #SHOOTING}.
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
		 * Currently tracking and shooting at a target. Spindexer is spindexing.
		 * <p>
		 * Once the hopper is out of balls (defined as when the hopper beam break hasn't detected a ball for {@link ShootingConstants#SHOOTING_TIMEOUT}), this will automatically transition back to {@link #HOME} state.
		 */
		SHOOTING_STAGE_3_SHOOTING,
		/**
		 * We were {@link #SHOOTING_STAGE_3_SHOOTING shooting}, but for whatever reason we had to stop to wait on a mechanism to ready (maybe the turret reached a limit and had to wrap around).
		 * <p>
		 * Once we're ready again, we'll immediately transition back to {@link #SHOOTING_STAGE_3_SHOOTING}.
		 */
		SHOOTING_PAUSED,
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
	 * @param uptakeBeamBreak
	 *            The beam break for the uptake.
	 */
	public ShooterSuperstructure(CommandSwerveDrivetrain drivetrain, StubbedFlywheel shooter, StubbedHood hood, StubbedTurret turret, StubbedHopperUptake hopperUptake, DigitalInput uptakeBeamBreak) {
		this.drivetrain = drivetrain;
		this.flywheel = shooter;
		this.hood = hood;
		this.turret = turret;
		this.hopperUptake = hopperUptake;
		this.uptakeBeamBreak = uptakeBeamBreak;

		// Set up the state machine
		// ---- Home state ----
		stateMachineConfig.configure(ShooterState.HOME)
				.onEntry(this::home)
				.permitReentry(ShooterTrigger.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.INITIALIZE, ShooterState.SHOOTING_STAGE_1_INITIALIZING);

		// ---- Manual control ----
		stateMachineConfig.configure(ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.HOME, ShooterState.HOME);

		// ---- Shooting state -----
		stateMachineConfig.configure(ShooterState.SHOOTING)
				// Spin up uptake while shooting
				.onEntry(hopperUptake::startUptakeForward)
				.onExit(hopperUptake::stopUptake)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_1_INITIALIZING)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.READY_TO_SHOOT, ShooterState.SHOOTING_STAGE_2_READY_TO_SHOOT);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_2_READY_TO_SHOOT)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING_STAGE_3_SHOOTING);

		stateMachineConfig.configure(ShooterState.SHOOTING_STAGE_3_SHOOTING)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.PAUSE_SHOOTING, ShooterState.SHOOTING_PAUSED)
				// Run hopper while shooting
				.onEntry(hopperUptake::startHopperForward)
				.onExit(hopperUptake::stopHopper);

		stateMachineConfig.configure(ShooterState.SHOOTING_PAUSED)
				.substateOf(ShooterState.SHOOTING)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING_STAGE_3_SHOOTING);

		stateMachine = new StateMachine<>(ShooterState.HOME, stateMachineConfig);

		stateMachine.onUnhandledTrigger((ShooterState state, ShooterTrigger trigger) -> {
			AlertUtil.sendNotification(AlertUtil.AlertLevel.ERROR, "Unhandled trigger", "Unhandled trigger: " + trigger + " from state: " + state, Seconds.of(3.0));
		});

		// Put commands on SmartDashboard
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Home", homeCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Start Manual Control", startManualControlCommand());
		SmartDashboard.putData(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Start Shooting", startShootingCommand());
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
		Pose2d robotPose = drivetrain.getPose2d();
		ChassisSpeeds robotVelocity = drivetrain.getFeildRelativeSpeeds();

		// Figure out target values if we have any
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(robotPose);

		Optional<ShooterValues> targetShooterValues = Optional.empty();
		if (targetPose.isPresent()) {
			targetShooterValues = Optional.of(ShootingCalculator.solve(new Pose3d(robotPose), targetPose.get(), robotVelocity));
		}

		// Check the current state and handle sequence transitions.
		switch (stateMachine.getState()) {
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

				// Once we reach the target, start to track
				if (subsystemsAtTargets()) {
					stateMachine.fire(ShooterTrigger.READY_TO_SHOOT);
					break;
				}

				// Get the shooter to an initial setpoint so we can start tracking
				if (targetShooterValues.isPresent()) {
					setValues(targetShooterValues.get(), false);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
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
				// Tracking the target and shooting

				// Shooter timeout
				if (!uptakeBeamBreak.get()) {
					shooterTimeout.restart();
					System.out.println("Reset the shooting timeout");
				} else {
					// After a little while of the beam break being off, stop the shooter
					if (shooterTimeout.hasElapsed(ShootingConstants.SHOOTING_TIMEOUT)) {
						stateMachine.fire(ShooterTrigger.HOME);

						shooterTimeout.stop();
						shooterTimeout.reset();

						break;
					}
				}

				if (!subsystemsAtTargets()) {
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

			case SHOOTING_PAUSED:
				// Was shooting, but needed to pause for subsystems to catch up

				if (subsystemsAtTargets()) {
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
	 * <p>
	 * Exits when exiting the {@link ShooterState#SHOOTING_STAGE_3_SHOOTING} state.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command startShootingCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.START_SHOOTING))
				.andThen(Commands.waitUntil(() -> !stateMachine.isInState(ShooterState.SHOOTING_STAGE_3_SHOOTING)));
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
				DriverStation.reportWarning("Attempted to execute setStateCommand without being in manual mode.", true);
			}
		}, flywheel, hood, turret);
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
	 * Checks if the subsystems are at their targets. This checks flywheel speed, hood position, and turret position.
	 *
	 * @return
	 *         Are all the shooter subsystems at their targets?
	 */
	public boolean subsystemsAtTargets() {
		return flywheel.isAtCruiseVelocity() && hood.isAtPosition() && turret.isAtTarget();
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
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(drivetrain.getPose2d());

		if (targetPose.isPresent()) {
			return targetPose.get();
		} else {
			return Pose3d.kZero;
		}
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
		flywheel.startFlywheel(values.getFlywheelSpeed());
		hood.moveToPosition(values.getHoodAngle());
		turret.setPosition(values.getTurretAngle(), !tracking);
	}

	/**
	 * Homes the various components of the turret.
	 */
	private void home() {
		// TODO: Update this once subsystems are in place
		flywheel.startFlywheel(RPM.of(10));
		hood.moveToPosition(Degrees.of(90));
		turret.unspool();

		// Temporary debug stuff
		System.out.println("Homed shooter");
	}
}
