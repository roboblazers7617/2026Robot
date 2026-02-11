package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Some static utility methods for calculating shooter values.
 */
public class ShootingCalculator {
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
	public static ShooterValues solve(Pose3d robotPose, Pose3d targetPose) {
		ShooterValues values = new ShooterValues();

		// TODO: Offset this according to turret position on the robot
		Pose3d turretPose = robotPose.transformBy(TurretConstants.TURRET_OFFSET);

		Transform3d gamepieceTransform = turretPose.minus(turretPose);

		// Solve shooter values
		values.setTurretAngle(solveTurretAngle(turretPose.toPose2d(), targetPose.toPose2d()));
		values.setHoodAngle(solveHoodAngle(gamepieceTransform));
		values.setFlywheelSpeed(solveGamepieceSpeed(gamepieceTransform));

		return values;
	}

	/**
	 * Solves the angle that the turret needs to face to point it at the targetPose.
	 *
	 * @param turretPose
	 *            The pose of the turret.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting Angle to point the turret at.
	 */
	private static Angle solveTurretAngle(Pose2d turretPose, Pose2d targetPose) {
		return turretPose.getTranslation()
				.minus(targetPose.getTranslation())
				.getAngle()
				.rotateBy(Rotation2d.k180deg)
				.getMeasure();
	}

	/**
	 * Solves the hood angle required to shoot the gamepiece to the desired transform.
	 *
	 * @param gamepieceTransform
	 *            The transform to shoot the gamepiece to.
	 * @return
	 *         The resulting Angle to angle the hood at.
	 */
	private static Angle solveHoodAngle(Transform3d gamepieceTransform) {
		// TODO: uhhh fancy math things?
		return Radians.zero();
	}

	/**
	 * Solves the gamepiece speed required to shoot the gamepiece to the desired transform.
	 *
	 * @param gamepieceTransform
	 *            The transform to shoot the gamepiece to.
	 * @return
	 *         The resulting LinearVelocity to shoot the gamepiece at.
	 */
	private static LinearVelocity solveGamepieceSpeed(Transform3d gamepieceTransform) {
		// TODO: more fancy math things!
		return MetersPerSecond.zero();
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
