// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	private final static String FRONT_CAMERA_NAME = "Arducam";
	private static final Transform3d ROBOT_TO_FRONT_CAM_TRANSFORM = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
	private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
	private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

	private final CommandSwerveDrivetrain drivetrain;
	private static final AprilTagFieldLayout AT_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

	private final PhotonCamera frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
	private final PhotonPoseEstimator photonFrontEstimator;

	private Matrix<N3, N1> curStdDevs;

	/** Creates a new Vision. */
	public Vision(CommandSwerveDrivetrain drivetrain) {
		this.drivetrain = drivetrain;
		photonFrontEstimator = new PhotonPoseEstimator(AT_FIELD_LAYOUT, ROBOT_TO_FRONT_CAM_TRANSFORM);
	}

	@Override
	public void periodic() {
		Optional<EstimatedRobotPose> frontVisionEst = Optional.empty();
		for (var result : frontCamera.getAllUnreadResults()) {
			frontVisionEst = photonFrontEstimator.estimateCoprocMultiTagPose(result);
			if (frontVisionEst.isEmpty()) {
				frontVisionEst = photonFrontEstimator.estimateLowestAmbiguityPose(result);
			}
			updateEstimationStdDevs(frontVisionEst, result.getTargets());

			if (frontVisionEst.isPresent()) {
				var est = frontVisionEst.get();

				// Change our trust in the measurement based on the tags we can see
				var estStdDevs = getEstimationStdDevs();

				drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
			}
		}
	}

	/**
	 * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
	 * deviations based on number of tags, estimation strategy, and distance from the tags.
	 *
	 * @param estimatedPose
	 *            The estimated pose to guess standard deviations for.
	 * @param targets
	 *            All targets in this camera frame
	 */
	private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
		if (estimatedPose.isEmpty()) {
			// No pose input. Default to single-tag std devs
			curStdDevs = SINGLE_TAG_STD_DEVS;
		} else {
			// Pose present. Start running Heuristic
			var estStdDevs = SINGLE_TAG_STD_DEVS;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we found, and calculate an average-distance metric
			for (var tgt : targets) {
				var tagPose = photonFrontEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
				if (tagPose.isEmpty())
					continue;
				numTags++;
				avgDist += tagPose
						.get()
						.toPose2d()
						.getTranslation()
						.getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
			}

			if (numTags == 0) {
				// No tags visible. Default to single-tag std devs
				curStdDevs = SINGLE_TAG_STD_DEVS;
			} else {
				// One or more tags visible, run the full heuristic.
				avgDist /= numTags;
				// Decrease std devs if multiple targets are visible
				if (numTags > 1)
					estStdDevs = MULTI_TAG_STD_DEVS;
				// Increase std devs based on (average) distance
				if (numTags == 1 && avgDist > 4)
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else
					estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
				curStdDevs = estStdDevs;
			}
		}
	}

	/**
	 * Returns the latest standard deviations of the estimated pose from {@link
	 * #getEstimatedGlobalPose()}, for use with {@link
	 * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
	 * only be used when there are targets visible.
	 */
	public Matrix<N3, N1> getEstimationStdDevs() {
		return curStdDevs;
	}
}
