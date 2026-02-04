package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Meters;

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

		// TODO: Offset this according to turret position on the robot
		Pose2d turretPose = robotPose;

		// TODO: Figure out what 2d pose we need to hit the 3d input in the travel path
		Pose2d target2d = targetPose.toPose2d();

		Distance shootingDistance = Meters.of(turretPose.getTranslation()
				.getDistance(target2d.getTranslation()));

		// Solve shooter values
		values.setTurretAngle(solveTurretAngle(turretPose, target2d));
		values.setHoodAngle(solveHoodAngle(turretPose, shootingDistance));
		values.setShooterSpeed(solveShooterSpeed(turretPose, shootingDistance));

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
	 * Solves the hood angle required to hit the specified targetPose on the ground.
	 *
	 * @param turretPose
	 *            The pose of the turret.
	 * @param targetPose
	 *            The distance to the target
	 * @return
	 *         The resulting Angle to angle the hood at.
	 */
	private static Angle solveHoodAngle(Pose2d turretPose, Distance targetDistance) {
		return Radians.of(ShootingConstants.hoodInterpolationTable.get(targetDistance.in(Meters)));
	}

	/**
	 * Solves the shooter speed required to hit the specified targetPose on the ground.
	 *
	 * @param turretPose
	 *            The pose of the turret.
	 * @param targetPose
	 *            The distance to the target
	 * @return
	 *         The resulting AngularVelocity to set the shooter to.
	 */
	private static AngularVelocity solveShooterSpeed(Pose2d turretPose, Distance targetDistance) {
		return RadiansPerSecond.of(ShootingConstants.shooterInterpolationTable.get(targetDistance.in(Meters)));
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
