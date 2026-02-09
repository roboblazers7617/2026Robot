package frc.robot.superstructure;

import com.github.oxo42.stateless4j.StateMachine;
import com.github.oxo42.stateless4j.StateMachineConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.StubbedIntakeGrabber;
import frc.robot.subsystems.StubbedIntakeShoulder;

/**
 * A superstructure that controls the functionality of the intake. This helps coordinate the various subsystems involved with the intake.
 */
@Logged
public class IntakeSuperstructure {
	// TODO: Implement things here for the various subsystems once those are added
	private final StubbedIntakeShoulder shoulder;
	private final StubbedIntakeGrabber grabber;

	/**
	 * The list of states the intake can be in.
	 */
	enum IntakeState {
		/**
		 * Homed and waiting for a command.
		 */
		HOME,
		/**
		 * Intaking from the field.
		 */
		INTAKING,
	}

	/**
	 * Triggers for the intake's state machine.
	 */
	enum IntakeTrigger {
		/**
		 * Stop and home the intake.
		 */
		HOME,
		/**
		 * Start the intake.
		 */
		INTAKE,
	}

	/**
	 * The configuration for the state machine.
	 */
	private final StateMachineConfig<IntakeState, IntakeTrigger> stateMachineConfig = new StateMachineConfig<>();
	/**
	 * The state machine.
	 */
	private final StateMachine<IntakeState, IntakeTrigger> stateMachine;

	/**
	 * Creates a new IntakeSuperstructure.
	 *
	 * @param shoulder
	 *            The shoulder to control.
	 * @param grabber
	 *            The grabber to control.
	 */
	public IntakeSuperstructure(StubbedIntakeShoulder shoulder, StubbedIntakeGrabber grabber) {
		this.shoulder = shoulder;
		this.grabber = grabber;

		// Set up the state machine
		stateMachineConfig.configure(IntakeState.HOME)
				.onEntry(shoulder::raiseShoulder)
				.onEntry(grabber::stopIntake)
				.permit(IntakeTrigger.INTAKE, IntakeState.INTAKING);

		stateMachineConfig.configure(IntakeState.INTAKING)
				.onEntry(shoulder::lowerShoulder)
				.onEntry(grabber::startIntake)
				.permit(IntakeTrigger.HOME, IntakeState.HOME);

		stateMachine = new StateMachine<>(IntakeState.HOME, stateMachineConfig);

		stateMachine.onUnhandledTrigger((IntakeState state, IntakeTrigger trigger) -> {
			DriverStation.reportError("Unhandled trigger: " + trigger + " from state: " + state, true);
		});

		// Put commands on SmartDashboard
		SmartDashboard.putData(SuperstructureConstants.INTAKE_SUPERSTRUCTURE_TABLE_NAME + "/Home", homeCommand());
		SmartDashboard.putData(SuperstructureConstants.INTAKE_SUPERSTRUCTURE_TABLE_NAME + "/Intake", intakeCommand());
	}

	/**
	 * Command to home the intake. This can be called from any state.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command homeCommand() {
		return Commands.runOnce(() -> stateMachine.fire(IntakeTrigger.HOME))
				.ignoringDisable(true);
	}

	/**
	 * Command to start the intake.
	 * <p>
	 * Can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command intakeCommand() {
		return Commands.runOnce(() -> stateMachine.fire(IntakeTrigger.INTAKE));
	}

	/**
	 * Checks if the subsystems are at their targets. This checks shoulder position.
	 *
	 * @return
	 *         Are all the intake subsystems at their targets?
	 */
	public boolean subsystemsAtTargets() {
		return shoulder.isAtTarget();
	}

	/**
	 * Gets the current state of the intake state machine.
	 */
	public IntakeState getState() {
		return stateMachine.getState();
	}
}
