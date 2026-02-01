package frc.robot.superstructure;

import java.util.Optional;
import java.util.function.Supplier;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A superstructure that controls the functionality of the shooter. This helps coordinate the various subsystems involved with the shooter.
 */
@Logged
public class ShooterController {
	// TODO: Implement things here for the various subsystems once those are added
	private Pose2d testRobotPose = new Pose2d(6.0, 1.0, Rotation2d.kZero);

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
	 */
	public ShooterController() {
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

		// TODO: Update once feeder is in place
		stateMachineConfig.configure(ShooterState.SHOOTING)
				// .onEntry(feeder::start)
				// .onExit(feeder::stop)
				.permit(ShooterTrigger.HOME, ShooterState.HOME)
				.permit(ShooterTrigger.START_MANUAL_CONTROL, ShooterState.MANUAL_CONTROL);

		stateMachine = new StateMachine<>(ShooterState.HOME, stateMachineConfig);

		// Put commands on SmartDashboard
		SmartDashboard.putData("Shooter Controller/Home", homeCommand());
		SmartDashboard.putData("Shooter Controller/Start Manual Control", startManualControlCommand());
	}

	/**
	 * Updates the shooter state depending on drivetrain position.
	 */
	public void update() {
		Optional<Pose3d> targetPose = ShootFromAnywhere.getTargetPoseForPosition(testRobotPose);

		// Check the current state and handle sequence transitions.
		switch (stateMachine.getState()) {
			case HOME:
				if (targetPose.isPresent()) {
					stateMachine.fire(ShooterTrigger.INITIALIZE);
				}
				break;

			case MANUAL_CONTROL:
				break;

			case INITIALIZING:
				// Once we reach the target, start to shoot
				if (isAtTarget()) {
					stateMachine.fire(ShooterTrigger.INITIALIZING_DONE);
					break;
				}

				// Get the shooter to an initial setpoint with MotionMagic
				if (targetPose.isPresent()) {
					prepareShootAtTarget(targetPose.get(), false);
				} else {
					stateMachine.fire(ShooterTrigger.HOME);
				}
				break;

			case SHOOTING:
				// Track the target without MotionMagic
				if (targetPose.isPresent()) {
					prepareShootAtTarget(targetPose.get(), true);
				} else {
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
		// TODO: Depend on all the subsystems involved once they are in here
		return Commands.run(() -> {
			if (stateMachine.isInState(ShooterState.MANUAL_CONTROL)) {
				setValues(values.get(), false);
			} else {
				DriverStation.reportWarning("[ShooterController] Attempted to execute setStateCommand without being in manual mode.", true);
			}
		});
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
		// TODO: Depend on all the subsystems involved once they are in here
		return Commands.run(() -> {
			if (stateMachine.isInState(ShooterState.MANUAL_CONTROL)) {
				prepareShootAtTarget(targetPose.get(), false);
			} else {
				DriverStation.reportWarning("[ShooterController] Attempted to execute prepareShootAtTargetCommand without being in manual mode.", true);
			}
		});
	}

	/**
	 * Checks if the subsystems are at their targets.
	 *
	 * @return
	 *         Are all the shooter subsystems at their targets?
	 */
	public boolean isAtTarget() {
		// TODO: Add logic here to check if the subsystems are at their targets
		return true;
	}

	/**
	 * Gets the current state of the shooter state machine.
	 */
	public ShooterState getState() {
		return stateMachine.getState();
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
		// TODO: Update this once drivetrain is in place
		// Pose2d robotPose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());
		Pose2d robotPose = new Pose2d();

		setValues(ShootFromAnywhere.solve(robotPose, targetPose), tracking);
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
		// shooter.setSpeed(values.getShooterSpeed());
		// turret.setAngle(values.getTurretAngle(), !tracking);
		// hood.setAngle(values.getHoodAngle());
	}

	/**
	 * Homes the various components of the turret.
	 */
	private void home() {
		// TODO: Update this once subsystems are in place
		// shooter.setSpeed(ShooterConstants.IDLE_SPEED);
		// turret.unspool();
		// hood.setAngle(HoodConstants.HOME_ANGLE);
	}
}
