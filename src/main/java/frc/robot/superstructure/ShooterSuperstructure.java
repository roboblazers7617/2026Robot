package frc.robot.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.StubbedHood;
import frc.robot.subsystems.StubbedHopperUptake;
import frc.robot.subsystems.StubbedFlywheel;
import frc.robot.subsystems.StubbedTurret;

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
		 * Initialized and ready to shoot.
		 */
		READY_TO_SHOOT,
		/**
		 * Currently tracking and shooting at a target.
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
		 * Signals to the superstructure that we are ready to shoot.
		 */
		READY_TO_SHOOT,
		/**
		 * Signals to the shooter to start shooting.
		 */
		START_SHOOTING,
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
		stateMachineConfig.configure(ShooterState.HOME)
				.onEntry(this::home)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.INITIALIZE, ShooterState.INITIALIZING);

		stateMachineConfig.configure(ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.HOME, ShooterState.HOME);

		stateMachineConfig.configure(ShooterState.INITIALIZING)
				// Spin up uptake while initializing
				.onEntry(hopperUptake::startUptakeForward)
				.onExit(hopperUptake::stopUptake)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.READY_TO_SHOOT, ShooterState.READY_TO_SHOOT);

		stateMachineConfig.configure(ShooterState.READY_TO_SHOOT)
				// Spin up uptake while awaiting shoot command
				.onEntry(hopperUptake::startUptakeForward)
				.onExit(hopperUptake::stopUptake)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL)
				.permit(ShooterTrigger.START_SHOOTING, ShooterState.SHOOTING);

		stateMachineConfig.configure(ShooterState.SHOOTING)
				// Run uptake while shooting
				.onEntry(hopperUptake::startUptakeForward)
				.onExit(hopperUptake::stopUptake)
				// Run hopper while shooting
				.onEntry(hopperUptake::startHopperForward)
				.onExit(hopperUptake::stopHopper)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		stateMachine = new StateMachine<>(ShooterState.HOME, stateMachineConfig);

		stateMachine.onUnhandledTrigger((ShooterState state, ShooterTrigger trigger) -> {
			DriverStation.reportError("Unhandled trigger: " + trigger + " from state: " + state, true);
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

				// Once we reach the target, start to track
				if (subsystemsAtTargets()) {
					stateMachine.fire(ShooterTrigger.READY_TO_SHOOT);
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

			case READY_TO_SHOOT:
				// Tracking the target without MotionMagic

				// We don't use MotionMagic here because it should improve tracking accuracy
				if (targetPose.isPresent()) {
					prepareShootAtTarget(targetPose.get(), true);
				} else {
					// If we don't have a target anymore, home the shooter
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;

			case SHOOTING:
				// Tracking the target without MotionMagic and shooting

				// Shooter timeout
				if (!uptakeBeamBreak.get()) {
					shooterTimeout.restart();
					System.out.println("Reset the shooting timeout");
				} else {
					// After a little while of the beam break being off, stop the shooter
					if (shooterTimeout.hasElapsed(Seconds.of(2.0))) {
						stateMachine.fire(ShooterTrigger.HOME);

						shooterTimeout.stop();
						shooterTimeout.reset();

						break;
					}
				}

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
	 * Command to start shooting. This can only be called from the INITIALIZING state.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command startShootingCommand() {
		return Commands.runOnce(() -> stateMachine.fire(ShooterTrigger.START_SHOOTING));
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
	 * Trigger that returns true when ready to shoot. Should be useful for triggering controller haptics.
	 *
	 * @return
	 *         Trigger, true when in {@link ShooterState#READY_TO_SHOOT}, false otherwise.
	 */
	public Trigger readyToShootTrigger() {
		return new Trigger(() -> stateMachine.isInState(ShooterState.READY_TO_SHOOT));
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
