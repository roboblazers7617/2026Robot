package frc.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.Elastic;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

/**
 * The turret that the shooter is attached to.
 */
@Logged
public class Turret extends SubsystemBase {
	/**
	 * The motor on the turret.
	 */
	private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID);

	/**
	 * The primary encoder on the turret. This is 1:1 with the motor, and {@link TurretConstants#PRIMARY_ENCODER_RATIO} to the mechanism.
	 */
	private final CANcoder primaryEncoder = new CANcoder(TurretConstants.PRIMARY_ENCODER_ID);
	/**
	 * The secondary encoder on the turret. This is {@link TurretConstants#SECONDARY_ENCODER_RATIO} to the mechanism.
	 */
	private final CANcoder secondaryEncoder = new CANcoder(TurretConstants.SECONDARY_ENCODER_ID);

	/**
	 * The configuration for the {@link #encoder}.
	 */
	private final CRTAbsoluteEncoderConfig encoderConfig = new CRTAbsoluteEncoderConfig(primaryEncoder.getPosition().asSupplier(), secondaryEncoder.getPosition().asSupplier())
			.withEncoderRatios(TurretConstants.PRIMARY_ENCODER_RATIO, TurretConstants.SECONDARY_ENCODER_RATIO)
			.withCrtGearRecommendationInputs(200, 0.1)
			.withCrtGearRecommendationConstraints(3, 5, 70, 10000);
	/**
	 * The encoder on the turret. This uses some gearing magic to derive the position based off the position of two absolute encoders.
	 */
	private final CRTAbsoluteEncoder encoder = new CRTAbsoluteEncoder(encoderConfig);

	/**
	 * The control request used for position control.
	 */
	private final PositionVoltage positionRequest = new PositionVoltage(0)
			.withSlot(0);
	/**
	 * The control request used for position control with MotionMagic.
	 */
	private final MotionMagicVoltage positionRequestMotionMagic = new MotionMagicVoltage(0)
			.withSlot(0);

	/**
	 * The current setpoint of the motor.
	 */
	private Angle setpoint = Rotations.of(0);

	/**
	 * Creates a new Turret.
	 */
	public Turret() {
		// If we're in simulation, calculate a gear ratio to use.
		if (Utils.isSimulation()) {
			Optional<CRTAbsoluteEncoderConfig.CrtGearPair> gearPair = encoderConfig.getRecommendedCrtGearPair();

			if (gearPair.isPresent()) {
				System.out.println(String.format("Gear 1: %d, Gear 2: %d", gearPair.get().gearA(), gearPair.get().gearB()));
			} else {
				System.out.println("Failed to solve optimal gear counts");
			}
		}

		TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = TurretConstants.MOTOR_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;
		talonFXConfigurator.apply(limitConfigs);

		// Sensor feedback configuration
		FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		feedbackConfigs.FeedbackRemoteSensorID = primaryEncoder.getDeviceID();
		feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		feedbackConfigs.SensorToMechanismRatio = TurretConstants.PRIMARY_ENCODER_RATIO;
		feedbackConfigs.RotorToSensorRatio = 1.0;
		talonFXConfigurator.apply(feedbackConfigs);

		// PID configuration
		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kS = TurretConstants.TURRET_KS;
		slot0Configs.kV = TurretConstants.TURRET_KV;
		slot0Configs.kA = TurretConstants.TURRET_KA;
		slot0Configs.kP = TurretConstants.TURRET_KP;
		slot0Configs.kI = TurretConstants.TURRET_KI;
		slot0Configs.kD = TurretConstants.TURRET_KD;
		talonFXConfigurator.apply(slot0Configs);

		// MotionMagic configuration
		MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
		motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.CRUISE_VELOCITY;
		motionMagicConfigs.MotionMagicAcceleration = TurretConstants.ACCELERATION;
		motionMagicConfigs.MotionMagicJerk = TurretConstants.JERK;
		talonFXConfigurator.apply(motionMagicConfigs);

		// Seed the encoder
		seedEncoder();
	}

	/**
	 * Command to set the turret to a certain position.
	 *
	 * @param position
	 *            Angle to turn to.
	 * @return
	 *         Command to run.
	 */
	public Command setPositionCommand(Supplier<Angle> position) {
		return run(() -> setPositionWithWrapping(position.get(), false));
	}

	/**
	 * Command to set the turret to a certain position with MotionMagic.
	 *
	 * @param position
	 *            Angle to turn to.
	 * @return
	 *         Command to run.
	 */
	public Command setPositionMotionMagicCommand(Supplier<Angle> position) {
		return run(() -> setPositionWithWrapping(position.get(), true));
	}

	/**
	 * Command to unspool the turret. This rotates back to an equivalent position +/- 0.5 rotations from center.
	 * <p>
	 * This allows you to "unspool" the mechanism, untwisting the wires.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command unspoolCommand() {
		return runOnce(() -> unspool());
	}

	/**
	 * Commands the turret motor closed loop controller directly to a certain position.
	 *
	 * @param position
	 *            Angle to turn to.
	 * @param motionMagic
	 *            Should MotionMagic be used?
	 */
	private void setPositionDirect(Angle position, boolean motionMagic) {
		if (motionMagic) {
			motor.setControl(positionRequestMotionMagic.withPosition(position));
		} else {
			motor.setControl(positionRequest.withPosition(position));
		}
		setpoint = position;
	}

	/**
	 * Commands the turret motor closed loop controller to a certain position, wrapping if applicable.
	 *
	 * @param position
	 *            Angle to turn to. Wrapped to be within [0-1] rotations (and the shortest path to the target is then taken).
	 * @param motionMagic
	 *            Should MotionMagic be used?
	 */
	private void setPositionWithWrapping(Angle position, boolean motionMagic) {
		setPositionDirect(findClosestTargetEquivalent(position), motionMagic);
	}

	/**
	 * Gets the current position of the turret with no wrapping. This reads the relative encoder on the motor.
	 *
	 * @return
	 *         The current position of the turret motor, with no wrapping and scaled to match gear ratios.
	 */
	public Angle getPositionDirect() {
		return motor.getRotorPosition().getValue();
	}

	/**
	 * Gets the current position of the turret, wrapped to stay within [0-1]. This reads the relative encoder on the motor.
	 *
	 * @return
	 *         The current position of the turret motor, wrapped and scaled to match gear ratios.
	 */
	public Angle getPosition() {
		double positionRadians = getPositionDirect().in(Radians);

		positionRadians %= Rotations.one().in(Radians);

		return Radians.of(positionRadians);
	}

	/**
	 * Seeds the turret encoder. This polls the absolute encoder value and uses it to set the relative encoder's position.
	 *
	 * @implNote
	 *           This may fail to solve! If it does, a notification will be sent to Elastic to notify the technician, since this is a fatal state to be in.
	 * @return
	 *         True if solved, false if failed to solve.
	 */
	private boolean seedEncoder() {
		// Encoder seeding
		Optional<Angle> turretPosition = encoder.getAngleOptional();
		if (turretPosition.isPresent()) {
			motor.setPosition(turretPosition.get());
		} else {
			Elastic.Notification notification = new Elastic.Notification()
					.withLevel(Elastic.Notification.NotificationLevel.ERROR)
					.withTitle("Failed to solve turret encoder position!")
					.withDescription("Failed to solve the turret encoder's position. This shouldn't happen! Try moving the turret a bit and restarting the robot code to see if it will solve correctly.");
			Elastic.sendNotification(notification);
			DriverStation.reportError("Failed to solve turret encoder position!", false);
		}

		return turretPosition.isPresent();
	}

	/**
	 * Find the closest equivalent angle to the target angle, within the range {@link TurretConstants#MINIMUM_ANGLE} to {@link TurretConstants#MAXIMUM_ANGLE}.
	 *
	 * @param target
	 *            The target position.
	 * @return
	 *         The angle to turn to.
	 */
	private Angle findClosestTargetEquivalent(Angle target) {
		double targetRotations = target.in(Rotations);
		double currentRotations = getPositionDirect().in(Rotations);

		// Wrap target position within one rotation
		targetRotations %= 1.0;

		// Handle negative remainder
		if (targetRotations < 0) {
			targetRotations += 1;
		}

		// How much can the new target differ from the current target?
		int kMin = (int) Math.ceil(TurretConstants.MINIMUM_ANGLE.in(Rotations) - targetRotations);
		int kMax = (int) Math.floor(TurretConstants.MAXIMUM_ANGLE.in(Rotations) - targetRotations);

		// Ideal difference (shortest path ignoring limits)
		int kIdeal = (int) Math.round(currentRotations - targetRotations);

		// Clamp to limits
		int k = Math.max(kMin, Math.min(kMax, kIdeal));

		// Final chosen absolute target
		return Rotations.of(targetRotations + k);
	}

	/**
	 * Rotates back to an equivalent position +/- 0.5 rotations from center.
	 * <p>
	 * This allows you to "unspool" the mechanism, untwisting the wires.
	 */
	public void unspool() {
		// Map current setpoint to a value within [0, 1]
		double targetRotations = ((setpoint.in(Rotations) + 0.5) % 1.0);

		// Handle negative remainder
		if (targetRotations < 0) {
			targetRotations += 1.0;
		}

		// Remap to [-0.5, 0.5]
		targetRotations -= 0.5;

		// Command the motor to spin
		setPositionDirect(Rotations.of(targetRotations), true);
	}
}
