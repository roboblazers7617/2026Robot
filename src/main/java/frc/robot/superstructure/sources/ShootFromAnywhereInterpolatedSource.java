package frc.robot.superstructure.sources;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.ShooterValues;
import frc.robot.superstructure.ShootingCalculator;

/**
 * A shooting source that dynamically adjusts to allow for shooting from anywhere, using interpolation tables internally.
 */
public class ShootFromAnywhereInterpolatedSource extends ShootingSource {
	private final CommandSwerveDrivetrain drivetrain;

	/**
	 * Creates a new ShootFromAnywhereInterpolatedSource.
	 *
	 * @param drivetrain
	 *            The drivetrain to poll for pose and velocity.
	 */
	public ShootFromAnywhereInterpolatedSource(CommandSwerveDrivetrain drivetrain) {
		super("Shoot from Anywhere");

		this.drivetrain = drivetrain;
	}

	@Override
	public Optional<ShooterValues> get() {
		Pose2d robotPose = drivetrain.getPose2d();

		// Figure out target values if we have any
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(robotPose);

		if (targetPose.isPresent()) {
			// If we have a target, solve some shooting values for it
			return Optional.of(ShootingCalculator.solveInterpolated(new Pose3d(robotPose), targetPose.get()));
		} else {
			return Optional.empty();
		}
	}
}
