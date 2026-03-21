package frc.robot.superstructure.sources;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import frc.robot.superstructure.ShooterValues;

/**
 * A source of values for the shooter.
 * <p>
 * This provides a nice way to keep track of different places the shooter can source its values.
 */
@Logged
public abstract class ShootingSource implements Supplier<Optional<ShooterValues>> {
	/**
	 * The name of the shooting source. This is mainly used for driver feedback.
	 */
	public final String name;

	/**
	 * Creates a new ShootingSource.
	 *
	 * @param name
	 *            The name of the shooting source.
	 */
	public ShootingSource(String name) {
		this.name = name;
	}

	/**
	 * Gets the name of the shooting source.
	 */
	public String getName() {
		return name;
	}
}
