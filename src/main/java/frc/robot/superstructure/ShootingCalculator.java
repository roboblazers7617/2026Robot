package frc.robot.superstructure;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.SuperstructureConstants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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
	public static String SHOOTING_CALCULATOR_MODELED_TABLE_NAME = SHOOTING_CALCULATOR_TABLE_NAME + "/Modeled";
	public static String SHOOTING_CALCULATOR_INTERPOLATED_TABLE_NAME = SHOOTING_CALCULATOR_TABLE_NAME + "/Interpolated";
	public static String SHOOTING_CALCULATOR_INTERPOLATED_WHILE_MOVE_TABLE_NAME = SHOOTING_CALCULATOR_TABLE_NAME + "/Interpolated While Move";

	private static StructPublisher<Pose3d> targetPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Target Pose", Pose3d.struct)
			.publish();
	private static DoublePublisher timeTillScorePublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Time Till Score")
			.publish();
	private static StructPublisher<Pose3d> modifiedTurretPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Modified Turret Pose", Pose3d.struct)
			.publish();
	private static StructPublisher<Translation2d> modifiedTranslationPublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Modified Translation", Translation2d.struct)
			.publish();
	private static StructPublisher<Pose3d> modifiedTargetPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Modified Target Pose", Pose3d.struct)
			.publish();
	private static StructPublisher<Pose3d> adjustedRobotPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Adjusted Robot Pose", Pose3d.struct)
			.publish();
	private static DoublePublisher gamepieceThetaPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Gamepiece Theta")
			.publish();
	private static DoublePublisher gamepieceSpeedPublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Gamepiece Speed")
			.publish();

	private static DoublePublisher distancePublisher = NetworkTableInstance.getDefault()
			.getDoubleTopic(SHOOTING_CALCULATOR_INTERPOLATED_TABLE_NAME + "/Distance")
			.publish();
	private static StructPublisher<Pose3d> interpolatedTargetPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_INTERPOLATED_TABLE_NAME + "/Target Pose", Pose3d.struct)
			.publish();
	private static StructPublisher<Pose3d> interpolatedTurretPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic(SHOOTING_CALCULATOR_INTERPOLATED_TABLE_NAME + "/Turret Pose", Pose3d.struct)
			.publish();

	private static StructArrayPublisher<Pose3d> interpolatedWhileMoveTurretPosePublisher = NetworkTableInstance.getDefault()
			.getStructArrayTopic(SHOOTING_CALCULATOR_INTERPOLATED_WHILE_MOVE_TABLE_NAME + "/Modified Turret Pose", Pose3d.struct)
			.publish();
	private static DoubleArrayPublisher interpolatedWhileMoveTimeTillScorePublisher = NetworkTableInstance.getDefault()
			.getDoubleArrayTopic(SHOOTING_CALCULATOR_INTERPOLATED_WHILE_MOVE_TABLE_NAME + "/Time Till Score")
			.publish();

	/**
	 * Solves shooter values for a given robot pose and target.
	 * <p>
	 * The math behind this is documented in <a href="https://www.desmos.com/calculator/s8yyqyc8iw">this Desmos calculator</a>.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @param robotVelocity
	 *            The robot-relative velocity of the robot.
	 * @return
	 *         The resulting values to apply.
	 */
	public static ShooterValues solve(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotVelocity) {
		ShooterValues values = new ShooterValues();

		targetPosePublisher.set(targetPose);
		SignalLogger.writeStruct(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Target Pose", Pose3d.struct, targetPose);

		// Figure out where the turret is since it isn't centered on the robot
		Pose3d exitPose = solveExitPose(robotPose, values.getTurretAngle(), values.getHoodAngle());

		// ----- FIRST CALCULATION (NO VELOCITY) -----
		// This stage is mainly just to calculate how long the ball will be in the air for

		// Solve the angle and translation to the target
		Translation2d gamepieceTranslation = solveGamepieceTranslation(exitPose, targetPose);

		// Solve initial shooter values
		Angle turretAngle = solveTurretAngle(exitPose.toPose2d(), targetPose.toPose2d());
		Angle gamepieceTheta = calculateHoodAngle(gamepieceTranslation);
		LinearVelocity gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);

		// ----- RE-CALCUlATION (WITH VELOCITY) -----
		for (int i = 0; i < SuperstructureConstants.SHOOTING_CALCULATOR_ITERATIONS; i++) {
			// Recalculate the exit pose of the ball
			exitPose = solveExitPose(robotPose, turretAngle, gamepieceTheta);

			// Figure out how long the gamepiece will be in the air for
			Time time = calculateTimeTillScore(gamepieceTranslation, gamepieceTheta, gamepieceSpeed);
			timeTillScorePublisher.set(time.in(Seconds));
			SignalLogger.writeValue(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Time Till Score", time);

			// Add the velocity vector of the robot
			// We're effectively trying to figure out where the turret will be at the end of the ball's travel
			Transform3d velocityAsTransform = new Transform3d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0.0, new Rotation3d());
			Pose3d modifiedTurretedPose = exitPose.transformBy(velocityAsTransform.times(time.in(Seconds)));
			modifiedTurretPosePublisher.set(modifiedTurretedPose);
			SignalLogger.writeStruct(SHOOTING_CALCULATOR_MODELED_TABLE_NAME + "/Modified Turret Pose", Pose3d.struct, modifiedTurretedPose);

			// Solve again, hood angle stays the same
			turretAngle = solveTurretAngle(modifiedTurretedPose.toPose2d(), targetPose.toPose2d());
			gamepieceTranslation = solveGamepieceTranslation(modifiedTurretedPose, targetPose);
			gamepieceTheta = calculateHoodAngle(gamepieceTranslation);
			gamepieceSpeed = solveGamepieceSpeed(gamepieceTranslation, gamepieceTheta);
		}

		modifiedTargetPosePublisher.set(exitPose.plus(new Transform3d(gamepieceTranslation.getX() * Math.cos(turretAngle.in(Radians)), gamepieceTranslation.getX() * Math.sin(turretAngle.in(Radians)), gamepieceTranslation.getY(), new Rotation3d())));
		modifiedTranslationPublisher.set(gamepieceTranslation);
		gamepieceThetaPublisher.set(gamepieceTheta.in(Radians));
		gamepieceSpeedPublisher.set(gamepieceSpeed.in(MetersPerSecond));

		// Set the ShooterValues accordingly
		values.setTurretAngle(turretAngle);
		values.setGamepieceTheta(gamepieceTheta);
		values.setGamepieceSpeed(gamepieceSpeed);

		// Set the ShooterValues into the simulation class
		// Theoretically this could be removed in favor of doing the gamepiece -> mechanism calculations backwards but I don't have time for that right now
		ShooterSim.setValues(gamepieceSpeed, gamepieceTheta);

		return values;
	}

	/**
	 * Solves shooter values for a given robot pose and target using interpolation.
	 *
	 * @param robotPose
	 *            The pose of the robot.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting values to apply.
	 */
	public static ShooterValues solveInterpolated(Pose3d robotPose, Pose3d targetPose) {
		ShooterValues values = new ShooterValues();

		interpolatedTargetPosePublisher.set(targetPose);

		// Figure out where the turret is since it isn't centered on the robot
		// The rotational component of this pose should just stay the rotation of the drivetrain
		Pose3d turretPose = robotPose.plus(SuperstructureConstants.ROBOT_TO_TURRET_BASE_TRANSFORM);

		interpolatedTurretPosePublisher.set(turretPose);

		// Solve the distance to the target
		Distance targetDistance = solveGamepieceTranslation(turretPose, targetPose).getMeasureX();

		distancePublisher.set(targetDistance.in(Meters));

		// Calculate the mechanism values
		Angle turretAngle = solveTurretAngle(turretPose.toPose2d(), targetPose.toPose2d());
		Angle hoodAngle = ShootingConstants.HOOD_ANGLE_BY_DISTANCE.get(targetDistance);
		AngularVelocity flywheelSpeed = ShootingConstants.FLYWHEEL_VELOCITY_BY_DISTANCE.get(targetDistance);

		// Set the ShooterValues accordingly
		values.setTurretAngle(turretAngle);
		values.setHoodAngle(hoodAngle);
		values.setFlywheelSpeed(flywheelSpeed);

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
		double a = 0.5 * Math.atan(-targetTranslation.getX() / targetTranslation.getY());

		if (a < 0) {
			a += (Math.PI / 2);
		}

		return Radians.of(MathUtil.clamp(a, ShootingConstants.MIN_SHOOT_ANGLE.in(Radians), ShootingConstants.MAX_SHOOT_ANGLE.in(Radians)));
	}

	/**
	 * Gets the pose where the ball will exit the shooter.
	 *
	 * @param robotPose
	 *            the pose of the robot
	 * @param thetaTurret
	 *            the angle of the turret, where 0 is facing towards the intake, ccw positive
	 * @param thetaHood
	 *            the angle of the shooter hood, ranging from 37 to 69 degrees
	 * @return
	 *         the pose of where the ball leaves the shooter
	 */
	public static Pose3d solveExitPose(Pose3d robotPose, Angle thetaTurret, Angle thetaHood) {
		// calculate the distance from the center of the turret pivot to where the ball is launched from
		Distance xHoodOffset = SuperstructureConstants.TURRET_BASE_TO_HOOD_PIVOT.getMeasureX().minus(SuperstructureConstants.HOOD_PIVOT_TO_GAMEPIECE_LAUNCH_RADIUS.times(Math.sin(thetaHood.in(Radians))));
		// use this to calculate the offset due to the hood and turret from the center of the turret pivot
		Distance xPos = SuperstructureConstants.ROBOT_TO_TURRET_BASE_TRANSFORM.getMeasureX().plus(xHoodOffset.times(Math.cos(thetaTurret.in(Radians))));
		Distance yPos = SuperstructureConstants.ROBOT_TO_TURRET_BASE_TRANSFORM.getMeasureY().plus(xHoodOffset.times(Math.sin(thetaTurret.in(Radians))));
		Distance zPos = SuperstructureConstants.ROBOT_TO_TURRET_BASE_TRANSFORM.getMeasureZ()
				.plus(SuperstructureConstants.TURRET_BASE_TO_HOOD_PIVOT.getMeasureZ())
				.plus(SuperstructureConstants.HOOD_PIVOT_TO_GAMEPIECE_LAUNCH_RADIUS.times(Math.cos(thetaHood.in(Radians))));
		// calculate the transform rotated by the robots pose
		// this assumes the robotpose ccw is positive

		// xMod = xcos(a) - ysin(a)
		Distance xModifiedPos = xPos.times(Math.cos(robotPose.getRotation().getZ())).minus(yPos.times(Math.sin(robotPose.getRotation().getZ())));
		// yMod = xsin(a)+ycos(a)
		Distance yModifiedPos = xPos.times(Math.sin(robotPose.getRotation().getZ())).plus(yPos.times(Math.cos(robotPose.getRotation().getZ())));

		Transform3d modifieTransformTurret = new Transform3d(xModifiedPos, yModifiedPos, zPos, new Rotation3d());
		return robotPose.plus(modifieTransformTurret);
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

		double dx = translation3d.getX();
		double dy = translation3d.getY();
		double dz = translation3d.getZ();

		double distance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));

		return new Translation2d(distance, dz);
	}

	/**
	 * Solves the angle between the turret and the target. This will be used to
	 * figure out the angle to point the turret at.
	 *
	 * @param turretPose
	 *            The pose of the turret. The translational component of this
	 *            should be the physical location of the turret, and the
	 *            rotational component should be the rotation of the
	 *            drivetrain.
	 * @param targetPose
	 *            The pose of the target to point at.
	 * @return
	 *         The resulting robot-relative Angle between the turret and the
	 *         target. This angle should just be directly set to the turret,
	 *         with no further modification to change between robot-relative
	 *         and field-relative.
	 */
	private static Angle solveTurretAngle(Pose2d turretPose, Pose2d targetPose) {
		return Radians.of(Math.atan2(targetPose.getY() - turretPose.getY(), targetPose.getX() - turretPose.getX()));
		// return targetPose.getTranslation()
		// .minus(turretPose.getTranslation())
		// .getAngle()
		// .getMeasure()
		// .minus(turretPose.getRotation().getMeasure());
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
		double a = .5 * ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond);
		double b = yVelocity;
		double c = -gamepieceTranslation.getY();
		System.out.println("A:" + a + " B:" + b + " C:" + c + " V:" + gamepieceSpeed.in(MetersPerSecond) + " Theta:" + gamepieceTheta.in(Degrees));
		// calculate quadratic formula to solve kinematics deltaY = Yinitial + Vyt + at^2
		double t = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
		return Seconds.of(t);
	}

	public static ShooterValues solveShootWhileMoveInterpolated(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotVelocity) {
		// these will be useful later, for now don't add the velocity vector of the robot
		Time time = Seconds.of(0);
		// we don't need to adjust for turret pose, as this is done during solveInterpolated
		Pose3d adjustedPose = robotPose;

		// find the values at the initial position
		ShooterValues curValue = solveInterpolated(adjustedPose, targetPose);

		// used for logging
		double[] timeTillScoreArray = new double[SuperstructureConstants.SHOOTING_CALCULATOR_ITERATIONS];
		Pose3d[] adjustedPoses = new Pose3d[SuperstructureConstants.SHOOTING_CALCULATOR_ITERATIONS];

		// iterate through the amount of calculations
		for (int i = 0; i < SuperstructureConstants.SHOOTING_CALCULATOR_ITERATIONS; i++) {
			// use a linreg to find the output velocity given by the lerp table
			LinearVelocity outputVelocity = MetersPerSecond.of(curValue.getFlywheelSpeed().in(RotationsPerSecond) * ShootingConstants.LINREG_FLYWHEEL_A + ShootingConstants.LINREG_FLYWHEEL_B);

			// use this to find the output angle assuming the ball perfectly hits the target
			Translation2d translationToTarget = solveGamepieceTranslation(solveExitPose(adjustedPose, curValue.getTurretAngle(), curValue.getHoodAngle()), targetPose);

			Angle outputAngle = solveOutputAngleFromVelocity(outputVelocity, translationToTarget);

			// use this to find the time till the ball lands
			System.out.println("FLYWHEEL:" + curValue.getFlywheelSpeed());
			time = calculateTimeTillScore(translationToTarget, outputAngle, outputVelocity);
			timeTillScoreArray[i] = time.in(Seconds);

			// add the robots velocity vector times the time
			Transform3d velocityAsTransform = new Transform3d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, 0.0, new Rotation3d());
			adjustedPose = robotPose.plus(velocityAsTransform.times(time.in(Seconds)));

			curValue = solveInterpolated(adjustedPose, targetPose);

			adjustedPoses[i] = adjustedPose;
			// System.out.println("Velocity:" + velocityAsTransform);
			System.out.println("Time:" + time.in(Seconds) + " Translation:" + translationToTarget);
		}
		adjustedRobotPosePublisher.set(adjustedPose);
		interpolatedWhileMoveTimeTillScorePublisher.set(timeTillScoreArray);
		interpolatedWhileMoveTurretPosePublisher.set(adjustedPoses);

		return curValue;
	}

	public static Angle solveOutputAngleFromVelocity(LinearVelocity outputVelocity, Translation2d translationToTarget) {
		// \arctan\left(\frac{\left(v^{2}+\sqrt{v^{4}-g^{2}d_{x}^{2}-2gv^{2}d_{y}}\right)}{g\cdot d_{x}}\right)
		// square v to make equations less cluttered
		double v = Math.pow(outputVelocity.in(MetersPerSecond), 2);
		double g = -ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond); // make g negative bc thats the formu
		double dx = translationToTarget.getMeasureX().in(Meters);
		double dy = translationToTarget.getMeasureY().in(Meters);

		// sqrt(v^4 - g^2dx^2-2gv^2dy)
		// EVERY OCCURENCE OF V IS SQUARED, so v^2 = outputVelocity ^4
		double sqrtTerm = Math.sqrt(Math.pow(v, 2) - Math.pow(g, 2) * Math.pow(dx, 2) - 2 * g * v * dy);
		System.out.println("v^2:" + v + " g:" + g + " dx:" + dx + " dy:" + dy);
		double innerTerm = (v + sqrtTerm) / (g * dx);
		Angle outputAngle = Radians.of(Math.atan(innerTerm));
		// 180-\ .5\left(\arccos\left(\frac{\frac{\left(g\cdot d_{x}^{2}\right)}{v^{2}}-d_{y}}{\sqrt{d_{y}^{2}+d_{x}^{2}}}\right)+\arctan\left(\frac{d_{x}}{d_{y}}\right)\right)
		// double g = ShootingConstants.GAMEPIECE_G.in(MetersPerSecondPerSecond);
		// double dx = translationToTarget.getMeasureX().in(Meters) + Units.inchesToMeters(8); // since we don't shoot in the exact middle, add an offset
		// double dy = translationToTarget.getMeasureY().in(Meters);

		// // calculate each term individualy, (tt/mt)/bt
		// double topTerm = g * Math.pow(dx, 2) - dy * Math.pow(outputVelocity.in(MetersPerSecond), 2);
		// double middleTerm = Math.pow(outputVelocity.in(MetersPerSecond), 2);
		// double bottomTerm = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

		// calculate the angle
		// there are two versions of this equation, the high and the low equations, the top equation is the low arc the bottom is the high arc
		// Angle outputAngle = Radians.of(.5 * (Math.acos((topTerm / middleTerm) / bottomTerm) - Math.atan(Math.abs(dx / dy))));
		// outputAngle = Degrees.of(90).minus(outputAngle);
		// Angle outputAngle = Radians.of(Math.PI - .5 * (Math.acos((topTerm / middleTerm) / bottomTerm) + Math.atan(Math.abs(dx / dy))));

		return outputAngle;
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
			if (DriverStation.getAlliance().get() == Alliance.Red) {
				return Optional.of(FieldConstants.ShootingZones.HUB_POSE.getRedPose());
			} else {
				if (robotPose.getMeasureY().compareTo(FieldConstants.FIELD_CENTER.getMeasureY()) < 0.0) {
					return Optional.of(FieldConstants.ShootingZones.SHUTTLE_CENTER_TOP_POSE);
				} else {
					return Optional.of(FieldConstants.ShootingZones.SHUTTLE_CENTER_BOTTOM_POSE);
				}
			}
		}

		if (FieldConstants.ShootingZones.HUB_ZONE_BLUE.contains(robotPose.getTranslation())) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				return Optional.of(FieldConstants.ShootingZones.HUB_POSE.getBluePose());
			} else {
				if (robotPose.getMeasureY().compareTo(FieldConstants.FIELD_CENTER.getMeasureY()) < 0.0) {
					return Optional.of(FieldConstants.ShootingZones.SHUTTLE_CENTER_TOP_POSE);
				} else {
					return Optional.of(FieldConstants.ShootingZones.SHUTTLE_CENTER_BOTTOM_POSE);
				}
			}
		}

		// Neutral zone
		if (FieldConstants.NEUTRAL_RECTANGLE.contains(robotPose.getTranslation())) {
			if (robotPose.getMeasureY().compareTo(FieldConstants.FIELD_CENTER.getMeasureY()) < 0.0) {
				return Optional.of(FieldConstants.ShootingZones.SHUTTLE_TOP_POSE.getPoseByAlliance());
			} else {
				return Optional.of(FieldConstants.ShootingZones.SHUTTLE_BOTTOM_POSE.getPoseByAlliance());
			}
		}

		return Optional.empty();
	}
}
