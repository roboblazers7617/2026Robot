package frc.robot.superstructure.sources;

import java.util.Optional;

import frc.robot.superstructure.ShooterValues;

/**
 * A shooting source that sets a constant value to the shooter.
 */
public class ShootingSourceConstant extends ShootingSource {
	private final ShooterValues values;

	/**
	 * Creates a new ShootingSourceConstant with the specified name and values.
	 *
	 * @param name
	 *            The name for the shooting source.
	 * @param values
	 *            The values to set.
	 */
	public ShootingSourceConstant(String name, ShooterValues values) {
		super(name);

		this.values = values;
	}

	@Override
	public Optional<ShooterValues> get() {
		return Optional.of(values);
	}
}
