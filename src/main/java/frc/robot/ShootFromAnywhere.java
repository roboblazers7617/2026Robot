package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.FieldConstants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;

/**
 * Math for the shoot from anywhere functionality of the robot.
 */
@Logged
public class ShootFromAnywhere {
	public Pose2d robotPose = FieldConstants.FIELD_LAYOUT.getTagPose(10)
			.get()
			.toPose2d()
			.transformBy(new Transform2d(Meters.of(3), Meters.of(-2), Rotation2d.kZero));
	public Pose2d hubPose = FieldConstants.FIELD_LAYOUT.getTagPose(10)
			.get()
			.toPose2d();
	public Angle testPose = solveTurretAngle(robotPose, hubPose);

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
