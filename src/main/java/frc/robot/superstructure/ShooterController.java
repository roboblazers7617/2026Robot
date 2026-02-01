package frc.robot.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A superstructure that controls the functionality of the shooter. This helps coordinate the various subsystems involved with the shooter.
 */
@Logged
public class ShooterController {
	// TODO: Implement things here for the various subsystems once those are added
	private Pose2d testRobotPose = new Pose2d(5.0, 1.0, Rotation2d.kZero);

	/**
	 * Creates a new ShooterController.
	 */
	public ShooterController() {
		System.out.println(ShootFromAnywhere.getTargetPoseForPosition(testRobotPose));
	}

	/**
	 * Sets the shooter state to the specified state.
	 * <p>
	 * This assumes that we are not tracking a target, since that is done internally.
	 *
	 * @param state
	 *            Supplier for the state to set the shooter to.
	 */
	public Command setStateCommand(Supplier<ShooterState> state) {
		// TODO: Depend on all the subsystems involved once they are in here
		return Commands.run(() -> setState(state.get(), false));
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
	 * @param tracking
	 *            Is the shooter currently tracking a target?
	 */
	private void setState(ShooterState state, boolean tracking) {
		// shooter.setSpeed(state.getShooterSpeed());
		// turret.setAngle(state.getTurretAngle(), !tracking);
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

		// setShooterState(ShootFromAnywhere.solve(robotPose, targetPose));
	}

	/**
	 * Homes the various components of the turret.
	 */
	private void home() {
		// shooter.setSpeed(ShooterConstants.IDLE_SPEED);
		// turret.unspool();
		// hood.setAngle(HoodConstants.HOME_ANGLE);
	}
}
