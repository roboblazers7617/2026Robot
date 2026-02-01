package frc.robot.superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Radians;

/**
 * An object that represents a state that the shooter can be in. This includes the shooter speed, turret position, and hood position.
 */
public class ShooterValues {
	private AngularVelocity shooterSpeed;
	private Angle turretAngle;
	private Angle hoodAngle;

	/**
	 * Creates a new ShooterValues with zero values.
	 */
	public ShooterValues() {
		shooterSpeed = RadiansPerSecond.of(0);
		turretAngle = Radians.of(0);
		hoodAngle = Radians.of(0);
	}

	/**
	 * Creates a new ShooterValues with specified values.
	 *
	 * @param shooterSpeed
	 *            The velocity to spin the shooter up to.
	 * @param turretAngle
	 *            The angle to point the turret at.
	 * @param hoodAngle
	 *            The angle to set the hood to.
	 */
	public ShooterValues(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
		this.shooterSpeed = shooterSpeed;
		this.turretAngle = turretAngle;
		this.hoodAngle = hoodAngle;
	}

	/**
	 * Sets the shooter speed.
	 *
	 * @param shooterSpeed
	 *            The velocity to spin the shooter up to.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setShooterSpeed(AngularVelocity shooterSpeed) {
		this.shooterSpeed = shooterSpeed;
		return this;
	}

	/**
	 * Sets the turret angle.
	 *
	 * @param turretAngle
	 *            The angle to point the turret at.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setTurretAngle(Angle turretAngle) {
		this.turretAngle = turretAngle;
		return this;
	}

	/**
	 * Sets the hood angle.
	 *
	 * @param hoodAngle
	 *            The angle to set the hood to.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setHoodAngle(Angle hoodAngle) {
		this.hoodAngle = hoodAngle;
		return this;
	}

	/**
	 * Gets the shooter speed.
	 *
	 * @return
	 *         The shooter speed.
	 */
	public AngularVelocity getShooterSpeed() {
		return shooterSpeed;
	}

	/**
	 * Gets the turret angle.
	 *
	 * @return
	 *         The turret angle.
	 */
	public Angle getTurretAngle() {
		return turretAngle;
	}

	/**
	 * Gets the hood angle.
	 *
	 * @return
	 *         The hood angle.
	 */
	public Angle getHoodAngle() {
		return hoodAngle;
	}
}
