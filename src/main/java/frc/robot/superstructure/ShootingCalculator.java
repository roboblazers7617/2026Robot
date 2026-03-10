package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TurretConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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

		Pose3d turretPose = robotPose.transformBy(TurretConstants.TURRET_OFFSET);

		// Solve the angle to the target
		Angle targetAngle = solveTargetAngle(turretPose.toPose2d(), targetPose.toPose2d());

		/**
		 * The transform from the turret to the target as a 2d plane where X is horizontal distance and Y is vertical distance.
		 */
		Translation2d gamepieceTranslation = solveGamepieceTranslation(turretPose, targetPose, targetAngle);

		// Solve shooter values
		Angle gamepieceTheta = calculateHoodAngle(gamepieceTranslation);
		LinearVelocity gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);

		// Solve with shoot while move
		Time time = calculateTimeTillScore(gamepieceTranslation, gamepieceTheta, gamepieceSpeed);

		// add the velocity vector of the robot
		ChassisSpeeds velocity = new ChassisSpeeds();

		Transform3d velocityAsTransform = new Transform3d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, 0.0, new Rotation3d());
		Pose3d modifiedTurretedPose = turretPose.transformBy(velocityAsTransform);
		// solve again, hood angle stays the same
		gamepieceTranslation = solveGamepieceTranslation(modifiedTurretedPose, targetPose, targetAngle);
		gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);

		values.setTurretAngle(targetAngle);
		values.setGamepieceTheta(gamepieceTheta);
		values.setFlywheelSpeed(gamepieceSpeed);

		return values;
	}

	public static Angle calculateHoodAngle(Translation2d targetTranslation) {
		Distance distance = Meters.of(Math.sqrt(Math.pow(targetTranslation.getX(), 2) + Math.pow(targetTranslation.getY(), 2)));

		if (distance.compareTo(ShootingConstants.CLOSE_SHOT_HOOD_CUTOFF) < 0) {
			return ShootingConstants.CLOSE_SHOT_HOOD_ANGLE;
		} else if (distance.compareTo(ShootingConstants.MEDIUM_SHOT_HOOD_CUTOFF) < 0) {
			return ShootingConstants.MEDIUM_SHOT_HOOD_ANGLE;
		} else if (distance.compareTo(ShootingConstants.FAR_SHOT_HOOD_CUTOFF) < 0) {
			return ShootingConstants.FAR_SHOT_HOOD_ANGLE;
		} else {
			// if more than far shot cutoff, go to max angle
			return ShootingConstants.MAX_HOOD_ANGLE;
		}
	}

	public static Translation2d solveGamepieceTranslation(Pose3d turretPose, Pose3d targetPose, Angle targetAngle) {
		return targetPose.minus(turretPose)
				.getTranslation()
				.rotateBy(new Rotation3d(Radians.zero(), Radians.zero(), targetAngle.unaryMinus()))
				.rotateBy(new Rotation3d(Rotations.of(0.75), Radians.zero(), Radians.zero()))
				.toTranslation2d();
	}

	/**
	 * Solves the angle between the turret and the target. This will be used to figure out the angle to point the turret at, as well as to transform the translation.
	 *
	 * @param turretPose
	 *            The pose of the turret.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting Angle between the turret and the target.
	 */
	private static Angle solveTargetAngle(Pose2d turretPose, Pose2d targetPose) {
		return turretPose.getTranslation()
				.minus(targetPose.getTranslation())
				.getAngle()
				.rotateBy(Rotation2d.k180deg)
				.getMeasure();
	}

	/**
	 * Solves the gamepiece speed required to shoot the gamepiece to the desired translation.
	 *
	 * @param gamepieceTranslation
	 *            The translation to shoot the gamepiece to.
	 * @param gamepieceTheta
	 *            The theta that the gamepiece is being shot at.
	 * @return
	 *         The resulting LinearVelocity to shoot the gamepiece at.
	 */
	private static LinearVelocity solveGamepieceSpeed(Translation2d gamepieceTranslation, Angle gamepieceTheta) {
		double deltaX = gamepieceTranslation.getX();
		double deltaY = gamepieceTranslation.getY();
		double g = ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond);
		double theta = gamepieceTheta.in(Radians);

		// @formatter:off
		double velocity = Math.sqrt(
				(g * Math.pow(deltaX, 2.0))
				/
				(2.0 * Math.pow(Math.cos(theta), 2.0) * (deltaY - (deltaX * Math.tan(theta))))
			);
		// @formatter:on

		return MetersPerSecond.of(velocity);
	}

	/**
	 * Calculates the time until the gamepiece hits the target
	 *
	 * @param gamepieceTranslation
	 *            The translation to the target
	 * @param gamepieceTheta
	 *            The theta the gamePiece will be fired at
	 * @param gamepieceSpeed
	 *            The speed the gamePiece will be fired at
	 */
	public static Time calculateTimeTillScore(Translation2d gamepieceTranslation, Angle gamepieceTheta, LinearVelocity gamepieceSpeed) {
		double yVelocity = gamepieceSpeed.in(MetersPerSecond) * Math.sin(gamepieceTheta.in(Radians));
		// quadratic formula values
		double a = ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond);
		double b = yVelocity;
		double c = -gamepieceTranslation.getY();
		// calculate quadratic formula to solve kinematics deltaY = Yinitial + Vyt + at^2
		double t = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
		return Seconds.of(t);
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
