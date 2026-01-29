package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class ShootFromAnywhere {
	/**
	 * Solves a shooter state for a given robot pose and target.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting ShooterState to apply.
	 */
	public static ShooterState solve(Pose2d robotPose, Pose3d targetPose) {
		ShooterState state = new ShooterState();

		// Turret
		state.setTurretAngle(solveTurretAngle(robotPose, targetPose.toPose2d()));

		// TOOD: Add shooter and hood

		return state;
	}

	/**
	 * Solves the angle that the turret needs to face to point it at the targetPose.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting Angle to point the turret at.
	 */
	private static Angle solveTurretAngle(Pose2d robotPose, Pose2d targetPose) {
		Pose2d turretPose = robotPose;

		return turretPose.getTranslation()
				.minus(targetPose.getTranslation())
				.getAngle()
				.rotateBy(Rotation2d.k180deg)
				.getMeasure();
	}
}
