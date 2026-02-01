package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.FieldConstants;

public class ShootFromAnywhere {
	/**
	 * Solves shooter values for a given robot pose and target.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting values to apply.
	 */
	public static ShooterValues solve(Pose2d robotPose, Pose3d targetPose) {
		ShooterValues values = new ShooterValues();

		// Turret
		values.setTurretAngle(solveTurretAngle(robotPose, targetPose.toPose2d()));

		// TOOD: Add shooter and hood

		return values;
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

	/**
	 * Gets the desired target pose for the given robot position.
	 *
	 * @param robotPose
	 *            The current pose of the robot.
	 * @return
	 *         Hub receptacle if viable, on to our side if in the center, or empty if otherwise.
	 */
	public static Optional<Pose3d> getTargetPoseForPosition(Pose2d robotPose) {
		// Hubs
		if (FieldConstants.ShootingZones.HUB_ZONE_RED.contains(robotPose.getTranslation())) {
			return Optional.of(FieldConstants.ShootingZones.HUB_POSE.getRedPose());
		}

		if (FieldConstants.ShootingZones.HUB_ZONE_BLUE.contains(robotPose.getTranslation())) {
			return Optional.of(FieldConstants.ShootingZones.HUB_POSE.getBluePose());
		}

		// Neutral zone
		if (FieldConstants.ShootingZones.NEUTRAL_ZONE_TOP.contains(robotPose.getTranslation())) {
			return Optional.of(FieldConstants.ShootingZones.SHUTTLE_TOP_POSE.getPoseByAlliance());
		}

		if (FieldConstants.ShootingZones.NEUTRAL_ZONE_BOTTOM.contains(robotPose.getTranslation())) {
			return Optional.of(FieldConstants.ShootingZones.SHUTTLE_BOTTOM_POSE.getPoseByAlliance());
		}

		return Optional.empty();
	}
}
