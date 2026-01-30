package frc.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.Elastic;

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
			.withEncoderRatios(TurretConstants.PRIMARY_ENCODER_RATIO, TurretConstants.SECONDARY_ENCODER_RATIO);
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
	 * Creates a new Turret.
	 */
	public Turret() {
		TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = TurretConstants.MOTOR_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;
		talonFXConfigurator.apply(limitConfigs);

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
	 */
	public Command setPositionCommand(Supplier<Angle> position) {
		return runOnce(() -> setPosition(position.get()));
	}

	/**
	 * Command to set the turret to a certain position with MotionMagic.
	 *
	 * @param position
	 *            Angle to turn to.
	 */
	public Command setPositionMotionMagicCommand(Supplier<Angle> position) {
		return runOnce(() -> setPositionMotionMagic(position.get()));
	}

	/**
	 * Commands the turret to a certain position.
	 *
	 * @param position
	 *            Angle to turn to.
	 */
	public void setPosition(Angle position) {
		motor.setControl(positionRequest.withPosition(position));
	}

	/**
	 * Commands the turret to a certain position with MotionMagic.
	 *
	 * @param position
	 *            Angle to turn to.
	 */
	public void setPositionMotionMagic(Angle position) {
		motor.setControl(positionRequestMotionMagic.withPosition(position));
	}

	/**
	 * Gets the current position of the turret. This reads the relative encoder on the motor.
	 *
	 * @return
	 *         The current position of the turret motor.
	 */
	public Angle getPosition() {
		return motor.getRotorPosition().getValue();
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
}
