package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * Utility classes for working with rectangles.
 */
public class RectangleUtil {
	/**
	 * Flips a rectangle to the other alliance.
	 *
	 * @param rectangle
	 *            The rectangle to flip.
	 * @return
	 *         The flipped rectangle.
	 */
	public static Rectangle2d flipRectangleX(Rectangle2d rectangle) {
		return new Rectangle2d(PoseUtil.flipPoseX(rectangle.getCenter()), rectangle.getMeasureXWidth(), rectangle.getMeasureYWidth());
	}

	/**
	 * Flips a rectangle top/bottom on the field.
	 *
	 * @param rectangle
	 *            The rectangle to flip.
	 * @return
	 *         The flipped rectangle.
	 */
	public static Rectangle2d flipRectangleY(Rectangle2d rectangle) {
		return new Rectangle2d(PoseUtil.flipPoseY(rectangle.getCenter()), rectangle.getMeasureXWidth(), rectangle.getMeasureYWidth());
	}

	/**
	 * Gets the top left corner of a rectangle.
	 *
	 * @param rectangle
	 *            The rectangle to use.
	 * @return
	 *         The pose of the top left corner.
	 */
	public static Pose2d getCornerTopLeft(Rectangle2d rectangle) {
		return rectangle.getCenter()
				.transformBy(new Transform2d(rectangle.getMeasureXWidth().div(2), rectangle.getMeasureYWidth().div(-2), Rotation2d.kZero));
	}

	/**
	 * Gets the top right corner of a rectangle.
	 *
	 * @param rectangle
	 *            The rectangle to use.
	 * @return
	 *         The pose of the top right corner.
	 */
	public static Pose2d getCornerTopRight(Rectangle2d rectangle) {
		return rectangle.getCenter()
				.transformBy(new Transform2d(rectangle.getMeasureXWidth().div(-2), rectangle.getMeasureYWidth().div(-2), Rotation2d.kZero));
	}

	/**
	 * Gets the bottom left corner of a rectangle.
	 *
	 * @param rectangle
	 *            The rectangle to use.
	 * @return
	 *         The pose of the bottom left corner.
	 */
	public static Pose2d getCornerBottomLeft(Rectangle2d rectangle) {
		return rectangle.getCenter()
				.transformBy(new Transform2d(rectangle.getMeasureXWidth().div(2), rectangle.getMeasureYWidth().div(2), Rotation2d.kZero));
	}

	/**
	 * Gets the bottom right corner of a rectangle.
	 *
	 * @param rectangle
	 *            The rectangle to use.
	 * @return
	 *         The pose of the bottom right corner.
	 */
	public static Pose2d getCornerBottomRight(Rectangle2d rectangle) {
		return rectangle.getCenter()
				.transformBy(new Transform2d(rectangle.getMeasureXWidth().div(-2), rectangle.getMeasureYWidth().div(2), Rotation2d.kZero));
	}

	/**
	 * Gets an array of the corners of a rectangle.
	 *
	 * @param rectangle
	 *            The rectangle to use.
	 * @return
	 *         An array of [top left, top right, bottom left, bottom right].
	 */
	public static Pose2d[] getCornerArray(Rectangle2d rectangle) {
		Pose2d[] poseList = { getCornerTopLeft(rectangle), getCornerTopRight(rectangle), getCornerBottomLeft(rectangle), getCornerBottomRight(rectangle) };

		return poseList;
	}
}
