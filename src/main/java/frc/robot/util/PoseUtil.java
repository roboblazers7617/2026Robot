package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;

/**
 * Utility classes for working with poses.
 */
public class PoseUtil {
	/**
	 * Flips a pose to the other alliance (inverts the X axis).
	 *
	 * @param pose
	 *            Pose to flip.
	 * @return
	 *         Flipped pose.
	 */
	public static Pose2d flipPoseX(Pose2d pose) {
		return new Pose2d(flipTranslationX(pose.getTranslation()), pose.getRotation());
	}

	/**
	 * Flips a pose to the other alliance (inverts the X axis).
	 *
	 * @param pose
	 *            Pose to flip.
	 * @return
	 *         Flipped pose.
	 */
	public static Pose3d flipPoseX(Pose3d pose) {
		return new Pose3d(flipTranslationX(pose.getTranslation()), pose.getRotation());
	}

	/**
	 * Flips a pose top/bottom on the field (inverts the Y axis).
	 *
	 * @param pose
	 *            Pose to flip.
	 * @return
	 *         Flipped pose.
	 */
	public static Pose2d flipPoseY(Pose2d pose) {
		return new Pose2d(flipTranslationY(pose.getTranslation()), pose.getRotation());
	}

	/**
	 * Flips a pose top/bottom on the field (inverts the Y axis).
	 *
	 * @param pose
	 *            Pose to flip.
	 * @return
	 *         Flipped pose.
	 */
	public static Pose3d flipPoseY(Pose3d pose) {
		return new Pose3d(flipTranslationY(pose.getTranslation()), pose.getRotation());
	}

	/**
	 * Flips a translation to the other alliance (inverts the X axis).
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation2d flipTranslationX(Translation2d translation) {
		return new Translation2d(FieldConstants.FIELD_LAYOUT.getFieldLength() - translation.getX(), translation.getY());
	}

	/**
	 * Flips a translation to the other alliance (inverts the X axis).
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation3d flipTranslationX(Translation3d translation) {
		return new Translation3d(FieldConstants.FIELD_LAYOUT.getFieldLength() - translation.getX(), translation.getY(), translation.getZ());
	}

	/**
	 * Flips a translaton top/bottom on the field (inverts the Y axis).
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation2d flipTranslationY(Translation2d translation) {
		return new Translation2d(translation.getX(), FieldConstants.FIELD_LAYOUT.getFieldWidth() - translation.getY());
	}

	/**
	 * Flips a translaton top/bottom on the field (inverts the Y axis).
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation3d flipTranslationY(Translation3d translation) {
		return new Translation3d(translation.getX(), FieldConstants.FIELD_LAYOUT.getFieldWidth() - translation.getY(), translation.getZ());
	}

	/**
	 * Flips a transform to the other alliance (inverts the X axis).
	 *
	 * @param transform
	 *            Transform to flip.
	 * @return
	 *         Flipped transform.
	 */
	public static Transform2d flipTransformY(Transform2d transform) {
		return new Transform2d(transform.getMeasureX(), transform.getMeasureY().times(-1), transform.getRotation());
	}

	/**
	 * Flips a transform to the other alliance (inverts the X axis).
	 *
	 * @param transform
	 *            Transform to flip.
	 * @return
	 *         Flipped transform.
	 */
	public static Transform3d flipTransformY(Transform3d transform) {
		return new Transform3d(transform.getMeasureX(), transform.getMeasureY().times(-1), transform.getMeasureZ(), transform.getRotation());
	}
}
