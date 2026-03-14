package frc.robot.subsystems.shooter;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.AlertUtil;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

/**
 * The turret that the shooter is attached to.
 *
 * @apiNote
 *          All set position methods in this class are robot-relative, since we don't want to interact with Drivetrain if we don't have to.
 */
@Logged
public class Turret extends SubsystemBase {
	/**
	 * The motor on the turret.
	 */
	private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID);

	/**
	 * The CANdi for the encoders.
	 * <p>
	 * PWM1 is the primary encoder on the turret. This is 1:1 with the motor, and {@link TurretConstants#PRIMARY_ENCODER_RATIO} to the mechanism.
	 * <p>
	 * PWM2 is the secondary encoder on the turret. This is {@link TurretConstants#SECONDARY_ENCODER_RATIO} to the mechanism.
	 */
	private final CANdi encoderCandi = new CANdi(TurretConstants.CANDI_ID);

	/**
	 * The configuration for the {@link #encoder}.
	 */
	private final EasyCRTConfig encoderConfig = new EasyCRTConfig(encoderCandi.getPWM2Position().asSupplier(), encoderCandi.getPWM1Position().asSupplier())
			.withEncoderRatios(TurretConstants.PRIMARY_ENCODER_RATIO, TurretConstants.SECONDARY_ENCODER_RATIO)
			.withCrtGearRecommendationInputs(200, 0.1)
			.withCrtGearRecommendationConstraints(3, 20, 70, 100);
	/**
	 * The encoder on the turret. This uses some gearing magic to derive the position based off the position of two absolute encoders.
	 */
	private final EasyCRT encoder = new EasyCRT(encoderConfig);

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
	 * The position of the simulated turret.
	 */
	private Angle simPosition = Radians.zero();

	/**
	 * The mechanism to use to publish turret information to NetworkTables.
	 */
	@Logged
	private Mechanism2d turretMechanism;
	/**
	 * The root component of the turret mechanism.
	 */
	private MechanismRoot2d turretMechanismRoot;
	/**
	 * The ligament of the turret mechanism that represents the actual turret rotation.
	 */
	private MechanismLigament2d turretMechanismLigament;

	/**
	 * Creates a new Turret.
	 */
	public Turret() {
		// If we're in simulation, calculate a gear ratio to use.
		if (Utils.isSimulation()) {
			Optional<EasyCRTConfig.CrtGearPair> gearPair = encoderConfig.getRecommendedCrtGearPair();

			if (gearPair.isPresent()) {
				System.out.println(gearPair.get());
			} else {
				System.out.println("Failed to solve optimal gear counts");
			}
		}

		TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

		// Motor output configuration
		talonFXConfiguration.MotorOutput
				.withNeutralMode(TurretConstants.MOTOR_NEUTRAL_MODE)
				.withInverted(TurretConstants.MOTOR_INVERTED);

		// Current limit configuration
		talonFXConfiguration.CurrentLimits
				.withSupplyCurrentLimit(TurretConstants.MOTOR_CURRENT_HIGHER_LIMIT)
				.withSupplyCurrentLowerLimit(TurretConstants.MOTOR_CURRENT_LOWER_LIMIT)
				.withSupplyCurrentLowerTime(TurretConstants.MOTOR_CURRENT_LOWER_TIME)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(TurretConstants.MOTOR_STATOR_CURRENT_LIMIT)
				.withStatorCurrentLimitEnable(true);

		// Sensor feedback configuration
		// TODO: Get this set up so the rotor to mechanism ratio works properly
		talonFXConfiguration.Feedback
				.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
				.withSensorToMechanismRatio(TurretConstants.MOTOR_GEAR_RATIO);

		// PID configuration
		talonFXConfiguration.Slot0
				.withKS(TurretConstants.TURRET_KS)
				.withKV(TurretConstants.TURRET_KV)
				.withKA(TurretConstants.TURRET_KA)
				.withKP(TurretConstants.TURRET_KP)
				.withKI(TurretConstants.TURRET_KI)
				.withKD(TurretConstants.TURRET_KD);

		// MotionMagic configuration
		talonFXConfiguration.MotionMagic
				.withMotionMagicCruiseVelocity(TurretConstants.CRUISE_VELOCITY)
				.withMotionMagicAcceleration(TurretConstants.ACCELERATION)
				.withMotionMagicJerk(TurretConstants.JERK);

		// Apply the configuration to the motor
		TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
		if (talonFXConfigurator.apply(talonFXConfiguration).isError()) {
			// If we run into an error, log it and notify on the dashboard
			AlertUtil.sendNotification(AlertUtil.AlertLevel.ERROR, "Failed to apply configuration to Turret motor", String.format("Failed to apply configuration to Turret motor (ID %d)! Make sure the CAN bus is operating properly.", TurretConstants.MOTOR_ID), Seconds.zero());
		}

		// Set up stuff for simulation
		if (Utils.isSimulation()) {
			TalonFXSimState talonFXSim = motor.getSimState();
			talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
			talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
		}

		// Set up the Mechanism2d if we're in debug mode
		if (LoggingConstants.DEBUG_MODE) {
			turretMechanism = new Mechanism2d(6, 6); // 6x6 units canvas
			turretMechanismRoot = turretMechanism.getRoot("TurretBase", 3, 3);

			turretMechanismLigament = new MechanismLigament2d("Turret", 2.0, 0.0);
			turretMechanismRoot.append(turretMechanismLigament);

			SmartDashboard.putData("TurretMechanism", turretMechanism);
		}

		// Seed the encoder
		seedEncoder();
	}

	@Override
	public void periodic() {
		// Update the Mechanism2d if we're in debug mode
		if (LoggingConstants.DEBUG_MODE) {
			turretMechanismLigament.setAngle(getPositionDirect().in(Degrees));
		}
	}

	@Override
	public void simulationPeriodic() {
		// Simulate proportional control to the setpoint
		// This is very basic simulation but it works well enough
		// Could maybe add some more advanced stuff later if needed
		double setpointRadians = setpoint.in(Radians);
		double simPositionRadians = simPosition.in(Radians);

		simPosition = Radians.of(MathUtil.interpolate(simPositionRadians, setpointRadians, 0.1));

		motor.getSimState().setRawRotorPosition(simPosition.times(TurretConstants.MOTOR_GEAR_RATIO));

		// Update simulated encoder values
		CANdiSimState encoderSimState = encoderCandi.getSimState();
		encoderSimState.setPwm1Position(simPosition.times(TurretConstants.PRIMARY_ENCODER_RATIO));
		encoderSimState.setPwm2Position(simPosition.times(TurretConstants.SECONDARY_ENCODER_RATIO));
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
	 * Gets the current position of the turret with no wrapping.
	 *
	 * @return
	 *         The current position of the turret, with no wrapping and scaled to match gear ratios.
	 */
	public Angle getPositionDirect() {
		return motor.getPosition()
				.getValue();
	}

	/**
	 * Gets the current position of the turret, wrapped to stay within [0-1].
	 *
	 * @return
	 *         The current position of the turret, wrapped and scaled to match gear ratios.
	 */
	public Angle getPosition() {
		double positionRadians = getPositionDirect().in(Radians);

		positionRadians %= Rotations.one().in(Radians);

		return Radians.of(positionRadians);
	}

	/**
	 * Checks if the turret is at its setpoint.
	 *
	 * @return
	 *         Is the turret at its setpoint?
	 */
	public boolean isAtTarget() {
		return getPositionDirect()
				.isNear(setpoint, TurretConstants.SETPOINT_THRESHOLD);
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
			AlertUtil.sendNotification(AlertUtil.AlertLevel.ERROR, "Failed to solve turret position!", "Failed to solve the turret's position. This shouldn't happen! Try moving the turret a bit and restarting the robot code to see if it will solve correctly.", Seconds.zero());
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
