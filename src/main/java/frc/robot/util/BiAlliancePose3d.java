package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * A Pose3d that contains versions for the Blue and Red alliances.
 */
public class BiAlliancePose3d {
	/**
	 * The pose on the blue alliance.
	 */
	public final Pose3d bluePose;
	/**
	 * The pose on the red alliance.
	 */
	public final Pose3d redPose;

	/**
	 * Changes the Y-Axis behavior when flipping the alliance.
	 */
	public enum InvertY {
		/**
		 * Keeps the Y value the same.
		 */
		KEEP_Y,
		/**
		 * Flips the Y value across the center line.
		 */
		INVERT_Y
	}

	/**
	 * Creates a BiAlliancePose3d from two poses.
	 *
	 * @param bluePose
	 *            The pose on the blue alliance.
	 * @param redPose
	 *            The pose on the red alliance.
	 */
	private BiAlliancePose3d(Pose3d bluePose, Pose3d redPose) {
		this.bluePose = bluePose;
		this.redPose = redPose;
	}

	/**
	 * Creates a BiAlliancePose3d from a pose on the blue alliance.
	 *
	 * @param bluePose
	 *            The pose on the blue alliance.
	 * @param invertY
	 *            Should the Y value be inverted?
	 */
	public static BiAlliancePose3d fromBluePose(Pose3d bluePose, InvertY invertY) {
		if (invertY == InvertY.KEEP_Y) {
			return new BiAlliancePose3d(bluePose, PoseUtil.flipPoseX(bluePose));
		} else {
			return new BiAlliancePose3d(bluePose, PoseUtil.flipPoseX(PoseUtil.flipPoseY(bluePose)));
		}
	}

	/**
	 * Creates a BiAlliancePose2d from a pose on the red alliance.
	 *
	 * @param redPose
	 *            The pose on the red alliance.
	 * @param invertY
	 *            Should the Y value be inverted?
	 */
	public static BiAlliancePose3d fromRedPose(Pose3d redPose, InvertY invertY) {
		if (invertY == InvertY.KEEP_Y) {
			return new BiAlliancePose3d(PoseUtil.flipPoseX(redPose), redPose);
		} else {
			return new BiAlliancePose3d(PoseUtil.flipPoseX(PoseUtil.flipPoseY(redPose)), redPose);
		}
	}

	/**
	 * Gets this pose on the blue alliance.
	 *
	 * @return
	 *         The pose on the blue alliance.
	 * @see #bluePose
	 */
	public Pose3d getBluePose() {
		return bluePose;
	}

	/**
	 * Gets this pose on the red alliance.
	 *
	 * @return
	 *         The pose on the red alliance.
	 * @see #redPose
	 */
	public Pose3d getRedPose() {
		return redPose;
	}

	/**
	 * Gets this pose for the current alliance.
	 *
	 * @return
	 *         The pose for the current alliance.
	 */
	public Pose3d getPoseByAlliance() {
		if (Util.isRedAlliance()) {
			return getRedPose();
		} else {
			return getBluePose();
		}
	}
}
