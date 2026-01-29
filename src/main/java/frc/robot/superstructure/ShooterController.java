package frc.robot.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A superstructure that controls the functionality of the shooter. This helps coordinate the various subsystems involved with the shooter.
 */
@Logged
public class ShooterController {
	// TODO: Implement things here for the various subsystems once those are added
	/**
	 * Sets the shooter state to the specified state.
	 *
	 * @param state
	 *            Supplier for the state to set the shooter to.
	 */
	public Command setShooterStateCommand(Supplier<ShooterState> state) {
		// TODO: Depend on all the subsystems involved once they are in here
		return Commands.run(() -> setShooterState(state.get()));
	}

	/**
	 * Command to prepare to shoot at a certain target pose from the drivetrain's current pose.
	 *
	 * @param targetPose
	 *            Supplier for the pose to shoot at.
	 */
	public Command prepareShootAtTargetCommand(Supplier<Pose3d> targetPose) {
		// TODO: Depend on all the subsystems involved once they are in here
		return Commands.run(() -> prepareShootAtTarget(targetPose.get()));
	}

	/**
	 * Sets the shooter state to the specified state.
	 *
	 * @param state
	 *            The state to set the shooter to.
	 */
	private void setShooterState(ShooterState state) {
		// shooter.setSpeed(state.getShooterSpeed());
		// turret.setAngle(state.getTurretAngle());
		// hood.setAngle(state.getHoodAngle());
	}

	/**
	 * Prepares to shoot at a certain target pose from the drivetrain's current pose.
	 *
	 * @param targetPose
	 *            The pose to shoot at.
	 */
	private void prepareShootAtTarget(Pose3d targetPose) {
		// Pose2d robotPose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

		// setShooterState(ShootWhileMove.computeShooterState(robotPose, targetPose));
	}
}
