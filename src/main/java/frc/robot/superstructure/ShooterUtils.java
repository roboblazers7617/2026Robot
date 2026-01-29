package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.FieldConstants;

/**
 * Utilities for controlling the shooter.
 */
public class ShooterUtils {
	/**
	 * Gets the desired target pose for the given robot position.
	 *
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
