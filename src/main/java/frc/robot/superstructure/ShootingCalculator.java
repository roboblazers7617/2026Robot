package frc.robot.superstructure;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TurretConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

/**
 * Some static utility methods for calculating shooter values.
 */
public class ShootingCalculator {
	/**
	 * The NetworkTables table name for the shooting calculator.
	 */
	public static String SHOOTING_CALCULATOR_TABLE_NAME = SuperstructureConstants.SHOOTER_SUPERSTRUCTURE_TABLE_NAME + "/Shooting Calculator";

	private static StructPublisher<Pose3d> targetPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Target Pose", Pose3d.struct)
			.publish();
	private static DoublePublisher timeTillScorePublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Time Till Score")
			.publish();
	private static StructPublisher<Pose3d> modifiedTurretPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Modified Turret Pose", Pose3d.struct)
			.publish();
	private static StructPublisher<Translation2d> modifiedTranslationPublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Modified Translation", Translation2d.struct)
			.publish();
	private static DoublePublisher gamepieceThetaPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Gamepiece Theta")
			.publish();
	private static DoublePublisher gamepieceSpeedPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_TABLE_NAME + "/Gamepiece Speed")
			.publish();

	/**
	 * Solves shooter values for a given robot pose and target.
	 * <p>
	 * The math behind this is documented in <a href="https://www.desmos.com/calculator/zixuz1clhy">this Desmos calculator</a>.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @param robotVelocity
	 *            The field-relative velocity of the robot.
	 * @return
	 *         The resulting values to apply.
	 */
	public static ShooterValues solve(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotVelocity) {
		ShooterValues values = new ShooterValues();

		targetPosePublisher.set(targetPose);
		SignalLogger.writeStruct(SHOOTING_CALCULATOR_TABLE_NAME + "/Target Pose", Pose3d.struct, targetPose);

		// Figure out where the turret is since it isn't centered on the robot
		Pose3d turretPose = robotPose.transformBy(TurretConstants.TURRET_OFFSET);

		// ----- FIRST CALCULATION (NO VELOCITY) -----
		// This stage is mainly just to calculate how long the ball will be in the air for

		// Solve the angle and translation to the target
		Angle targetAngle = solveTargetAngle(turretPose.toPose2d(), targetPose.toPose2d());
		Translation2d gamepieceTranslation = solveGamepieceTranslation(turretPose, targetPose);

		// Solve initial shooter values
		Angle gamepieceTheta = calculateHoodAngle(gamepieceTranslation);
		LinearVelocity gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);

		// ----- RE-CALCUlATION (WITH VELOCITY) -----
		for (int i = 0; i < SuperstructureConstants.SHOOTING_CALCULATOR_ITERATIONS; i++) {
			// Figure out how long the gamepiece will be in the air for
			Time time = calculateTimeTillScore(gamepieceTranslation, gamepieceTheta, gamepieceSpeed);
			timeTillScorePublisher.set(time.in(Seconds));
			SignalLogger.writeValue(SHOOTING_CALCULATOR_TABLE_NAME + "/Time Till Score", time);

			// Add the velocity vector of the robot
			// We're effectively trying to figure out where the turret will be at the end of the ball's travel
			Transform3d velocityAsTransform = new Transform3d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0.0, new Rotation3d());
			Pose3d modifiedTurretedPose = turretPose.transformBy(velocityAsTransform.times(time.in(Seconds)));
			modifiedTurretPosePublisher.set(modifiedTurretedPose);
			SignalLogger.writeStruct(SHOOTING_CALCULATOR_TABLE_NAME + "/Modified Turret Pose", Pose3d.struct, modifiedTurretedPose);

			// Solve again, hood angle stays the same
			targetAngle = solveTargetAngle(modifiedTurretedPose.toPose2d(), targetPose.toPose2d());
			gamepieceTranslation = solveGamepieceTranslation(modifiedTurretedPose, targetPose);
			gamepieceTheta = calculateHoodAngle(gamepieceTranslation);
			gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);
		}

		modifiedTranslationPublisher.set(gamepieceTranslation);
		gamepieceThetaPublisher.set(gamepieceTheta.in(Radians));
		gamepieceSpeedPublisher.set(gamepieceSpeed.in(MetersPerSecond));

		// Set the ShooterValues accordingly
		values.setTurretAngle(targetAngle);
		values.setGamepieceTheta(gamepieceTheta);
		values.setFlywheelSpeed(gamepieceSpeed);

		// Set the ShooterValues into the simulation class
		// Theoretically this could be removed in favor of doing the gamepiece -> mechanism calculations backwards but I don't have time for that right now
		ShooterSim.setValues(gamepieceSpeed, gamepieceTheta);

		return values;
	}

	/**
	 * Calculates the desired gamepiece theta for the given target translation.
	 *
	 * @param targetTranslation
	 *            The translation to shoot at.
	 * @return
	 *         The theta to shoot the gamepiece at.
	 */
	public static Angle calculateHoodAngle(Translation2d targetTranslation) {
		double a = 0.5 * Math.atan(-targetTranslation.getX() / targetTranslation.getY()) + (Math.PI / 2);

		return Radians.of(MathUtil.clamp(a, ShootingConstants.MIN_SHOOT_ANGLE.in(Radians), ShootingConstants.MAX_SHOOT_ANGLE.in(Radians)));
	}

	/**
	 * Gets the 2d translation from the turret to the target.
	 *
	 * @param turretPose
	 *            The pose of the turret.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The translation from the turret to the target as a 2d plane where X is horizontal distance and Y is vertical distance.
	 */
	public static Translation2d solveGamepieceTranslation(Pose3d turretPose, Pose3d targetPose) {
		Translation3d translation3d = targetPose.minus(turretPose)
				.getTranslation();

		double dx = Math.abs(translation3d.getX());
		double dy = Math.abs(translation3d.getY());
		double dz = translation3d.getZ();

		double distance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));

		return new Translation2d(distance, dz + 0.5);
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
	 * @return
	 *         The time it will take for the gamepiece to hit the target.
	 */
	public static Time calculateTimeTillScore(Translation2d gamepieceTranslation, Angle gamepieceTheta, LinearVelocity gamepieceSpeed) {
		double yVelocity = gamepieceSpeed.in(MetersPerSecond) * Math.sin(gamepieceTheta.in(Radians));
		// quadratic formula values
		double a = ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond);
		double b = yVelocity;
		double c = gamepieceTranslation.getY();
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
