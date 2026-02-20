package frc.robot.superstructure;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.SuperstructureConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
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
		SignalLogger.writeStruct(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Turret Pose", Pose3d.struct, turretPose);

		// Solve the turret rotation. This is done now because it's needed in a few different spots
		Angle turretRotation = solveTurretAngle(turretPose.toPose2d(), targetPose.toPose2d());

		/**
		 * The transform from the turret to the target as a 2d plane where X is horizontal distance and Y is vertical distance.
		 */
		Translation2d gamepieceTranslation = targetPose.minus(turretPose)
				.getTranslation()
				.rotateBy(new Rotation3d(Radians.zero(), Radians.zero(), turretRotation.unaryMinus()))
				.rotateBy(new Rotation3d(Rotations.of(0.75), Radians.zero(), Radians.zero()))
				.toTranslation2d();

		SignalLogger.writeStruct(SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Gamepiece Translation", Translation2d.struct, gamepieceTranslation);

		// Solve shooter values
		Angle gamepieceTheta = Degrees.of(80.0);
		LinearVelocity gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);

		values.setTurretAngle(turretRotation);
		values.setGamepieceTheta(gamepieceTheta);
		values.setFlywheelSpeed(gamepieceSpeed);

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
