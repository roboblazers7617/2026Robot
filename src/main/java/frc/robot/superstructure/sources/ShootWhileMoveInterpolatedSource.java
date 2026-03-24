package frc.robot.superstructure.sources;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.superstructure.ShooterValues;
import frc.robot.superstructure.ShootingCalculator;

/**
 * A shooting source that dynamically adjusts to allow for shooting from
 * anywhere, using interpolation tables internally. This also accounts for
 * drivetrain velocity, allowing for shooting on the move.
 */
public class ShootWhileMoveInterpolatedSource extends ShootingSource {
	private final CommandSwerveDrivetrain drivetrain;

	/**
	 * Creates a new ShootWhileMoveInterpolatedSource.
	 *
	 * @param drivetrain
	 *            The drivetrain to poll for pose and velocity.
	 */
	public ShootWhileMoveInterpolatedSource(CommandSwerveDrivetrain drivetrain) {
		super("Shoot from Anywhere");

		this.drivetrain = drivetrain;
	}

	@Override
	public Optional<ShooterValues> get() {
		Pose2d robotPose = drivetrain.getPose2d();
		ChassisSpeeds robotVelocity = drivetrain.getRobotRelativeSpeeds();

		// Figure out target values if we have any
		Optional<Pose3d> targetPose = ShootingCalculator.getTargetPoseForPosition(robotPose);

		if (targetPose.isPresent()) {
			// If we have a target, solve some shooting values for it
			return Optional.of(ShootingCalculator.solveShootWhileMoveInterpolated(new Pose3d(robotPose), targetPose.get(), robotVelocity));
		} else {
			return Optional.empty();
		}
	}
}
