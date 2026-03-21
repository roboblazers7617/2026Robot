package frc.robot.superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.ShootingConstants;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;

/**
 * An object that represents a state that the shooter can be in. This includes the flywheel speed, turret position, and hood position.
 */
public class ShooterValues {
	private AngularVelocity flywheelSpeed;
	private Angle turretAngle;
	private Angle hoodAngle;

	/**
	 * Creates a new ShooterValues with zero values.
	 */
	public ShooterValues() {
		flywheelSpeed = RadiansPerSecond.of(0);
		turretAngle = Radians.of(0);
		hoodAngle = Radians.of(0);
	}

	/**
	 * Creates a new ShooterValues with specified values.
	 *
	 * @param flywheelSpeed
	 *            The velocity to spin the flywheel up to.
	 * @param hoodAngle
	 *            The angle to set the hood to.
	 * @param turretAngle
	 *            The angle to point the turret at.
	 */
	public ShooterValues(AngularVelocity flywheelSpeed, Angle hoodAngle, Angle turretAngle) {
		this.flywheelSpeed = flywheelSpeed;
		this.turretAngle = turretAngle;
		this.hoodAngle = hoodAngle;
	}

	/**
	 * Sets the flywheel speed to a specific {@link AngularVelocity}.
	 *
	 * @param flywheelSpeed
	 *            The velocity to spin the flywheel up to.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setFlywheelSpeed(AngularVelocity flywheelSpeed) {
		this.flywheelSpeed = flywheelSpeed;
		return this;
	}

	/**
	 * Sets the flywheel speed as a {@link LinearVelocity} to shoot the gamepiece at (exit velocity).
	 *
	 * @param gamepieceSpeed
	 *            The velocity to shoot the gamepiece at.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setGamepieceSpeed(LinearVelocity gamepieceSpeed) {
		// this.flywheelSpeed = (gamepieceSpeed-ShootingConstants.LINREG_FLYWHEEL_B)/ShootingConstants.LINREG_FLYWHEEL_A;
		this.flywheelSpeed = ShootingConstants.FLYWHEEL_VELOCITY_BY_GAMEPIECE_VELOCITY.get(gamepieceSpeed);
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
	 * Sets the hood angle as an {@link Angle} to shoot the gamepiece at (exit angle).
	 *
	 * @param gamepieceTheta
	 *            The angle to shoot the gamepiece at.
	 * @return
	 *         This object for method chaining.
	 */
	public ShooterValues setGamepieceTheta(Angle gamepieceTheta) {
		// this.hoodAngle = (gamepieceTheta-ShootingConstants.LINREG_HOOD_ANGLE_B)/ShootingConstants.LINREG_HOOD_ANGLE_A;
		this.hoodAngle = ShootingConstants.HOOD_ANGLE_BY_GAMEPIECE_THETA.get(gamepieceTheta);
		return this;
	}

	/**
	 * Gets the flywheel speed.
	 *
	 * @return
	 *         The flywheel speed.
	 */
	public AngularVelocity getFlywheelSpeed() {
		return flywheelSpeed;
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

	@Override
	public String toString() {
		return String.format("Flywheel Speed: %f RPS, Hood Angle: %f Degrees, Turret Angle: %f Degrees", getFlywheelSpeed().in(RotationsPerSecond), getHoodAngle().in(Degrees), getTurretAngle().in(Degrees));
	}
}
