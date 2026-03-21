package frc.robot.superstructure.sources;

import java.util.Optional;

import frc.robot.superstructure.ShooterValues;

/**
 * A shooting source that never outputs a target.
 */
public class ShootingSourceIdle extends ShootingSource {
	/**
	 * Creates a new ShootingSourceIdle.
	 */
	public ShootingSourceIdle() {
		super("Idle");
	}

	@Override
	public Optional<ShooterValues> get() {
		return Optional.empty();
	}
}
