// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

public class Vision extends SubsystemBase {
	private final PhotonCamera turretCamera;
	private final PhotonPoseEstimator photonTurretEstimator;
	private final PhotonCamera eBoardCamera;
	private final PhotonPoseEstimator photonEBoardEstimator;
	private final CommandSwerveDrivetrain drivetrain;

	private Matrix<N3, N1> curStdDevs;

	/** Creates a new Vision. */
	public Vision(CommandSwerveDrivetrain drivetrain) {
		turretCamera = new PhotonCamera(VisionConstants.TURRET_CAM_NAME);
		photonTurretEstimator = new PhotonPoseEstimator(FieldConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_TURRET_CAM_TRANSFORM);
		eBoardCamera = new PhotonCamera(VisionConstants.EBOARD_CAM_NAME);
		photonEBoardEstimator = new PhotonPoseEstimator(FieldConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_EBOARD_CAM_TRANSFORM);
		this.drivetrain = drivetrain;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// front cam data yoinking
		Optional<EstimatedRobotPose> turretVisionEst = Optional.empty();
		for (var result : turretCamera.getAllUnreadResults()) {
			turretVisionEst = photonTurretEstimator.estimateCoprocMultiTagPose(result);
			if (turretVisionEst.isEmpty()) {
				turretVisionEst = photonTurretEstimator.estimateLowestAmbiguityPose(result);
			}
			updateEstimationStdDevs(turretVisionEst, result.getTargets(), photonTurretEstimator);

			if (turretVisionEst.isPresent()) {
				var est = turretVisionEst.get();
				var estStdDevs = getEstimationStdDevs();
				for (int tgt = 0; tgt < result.getTargets().size(); tgt++) {
					if (result.getTargets().get((tgt)).getPoseAmbiguity() <= 0.1) {
						drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
						if (VisionConstants.DEBUG_PRINT_STATEMENTS)
							System.out.println("turret measurement");
					} else {
						if (VisionConstants.DEBUG_PRINT_STATEMENTS)
							System.out.println("turret rejected");
					}
				}
			}
		}

		// side cam data yoinking
		Optional<EstimatedRobotPose> eBoardVisionEst = Optional.empty();
		for (var result : eBoardCamera.getAllUnreadResults()) {
			eBoardVisionEst = photonEBoardEstimator.estimateCoprocMultiTagPose(result);
			if (eBoardVisionEst.isEmpty()) {
				eBoardVisionEst = photonEBoardEstimator.estimateLowestAmbiguityPose(result);
			}
			updateEstimationStdDevs(eBoardVisionEst, result.getTargets(), photonEBoardEstimator);

			if (eBoardVisionEst.isPresent()) {
				var est = eBoardVisionEst.get();
				var estStdDevs = getEstimationStdDevs();
				for (int tgt = 0; tgt < result.getTargets().size(); tgt++) {
					if (result.getTargets().get((tgt)).getPoseAmbiguity() <= 0.1) {
						drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
						if (VisionConstants.DEBUG_PRINT_STATEMENTS)
							System.out.println("eboard measurement");
					} else {
						if (VisionConstants.DEBUG_PRINT_STATEMENTS)
							System.out.println("eboard rejected");
					}
				}
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
	private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonEitherEstimator) {
		if (estimatedPose.isEmpty()) {
			// No pose input. Default to single-tag std devs
			curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
		} else {
			// Pose present. Start running Heuristic
			var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
			int numTags = 0;
			double avgDist = 0;

			// Precalculation - see how many tags we found, and calculate an average-distance metric
			for (var tgt : targets) {
				var tagPose = photonEitherEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
				curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
			} else {
				// One or more tags visible, run the full heuristic.
				avgDist /= numTags;
				// Decrease std devs if multiple targets are visible
				if (numTags > 1)
					estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
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
	 * Returns the latest standard deviations of the estimated pose from
	 * {@link #periodic()}, for use with {@link
	 * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
	 * SwerveDrivePoseEstimator}. This should only be used when there are
	 * targets visible.
	 */
	public Matrix<N3, N1> getEstimationStdDevs() {
		return curStdDevs;
	}
}
