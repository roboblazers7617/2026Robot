package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.MotorMonitor;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeShoulder extends SubsystemBase {
	// controller for motor which moves intake
	private final TalonFX motor;

	private final MotionMagicVoltage positionRequestOut = new MotionMagicVoltage(0).withSlot(1);
	private final DynamicMotionMagicVoltage positionRequestIn = new DynamicMotionMagicVoltage(0, IntakeConstants.SLOW_MAXIMUM_VELOCITY, IntakeConstants.SLOW_ACCELERATION).withSlot(0);

	private final CANcoder intakeEncoder = new CANcoder(IntakeConstants.SHOULDER_ENCODER_CAN_ID, Constants.CANIVORE_BUS);

	private double setPointMeters;

	/**
	 * Constructor for motor which raises and lowers intakes with associated
	 * configurations, sets up Motion Magic, and configures gear ratios.
	 */
	public IntakeShoulder() {
		motor = new TalonFX(IntakeConstants.SHOULDER_CAN_ID, Constants.CANIVORE_BUS);

		// TODO: See the comments on IntakeGrabber on how to better code this
		// TalonFXConfigurator motorConfigurator = motor.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();

		// Current limit configuration
		// CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		// limitConfigs.SupplyCurrentLowerLimit =
		// IntakeConstants.SHOULDER_SUPPLY_CURRENT_LOWER_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LOWER_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LOWER_TIME;
		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.SHOULDER_STATOR_CURRENT_LIMIT;
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		// motorConfigurator.apply(limitConfigs);

		// MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		// Puts the motor in Coast mode to make it easier to move by hand
		// TODO: This needs to be in brake mode so it stays in place
		// motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Configure the motor to make sure positive voltage is counter clockwise
		motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		// motorConfigurator.apply(motorOutputConfigs);

		// Slot0Configs slot0Configs = new Slot0Configs();
		// slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
		motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static; // Use cosine gravity compensation
		motorConfig.Slot0.kG = IntakeConstants.INTAKE_KG_0; // Gravity gain
		motorConfig.Slot0.kS = IntakeConstants.INTAKE_KS_0; // Add ____ V output to overcome static friction
		motorConfig.Slot0.kV = IntakeConstants.INTAKE_KV_0; // A velocity target of 1 rps results in ____ V output
		motorConfig.Slot0.kA = IntakeConstants.INTAKE_KA_0; // An acceleration of 1 rps/s requires ____ V output
		motorConfig.Slot0.kP = IntakeConstants.INTAKE_KP_0; // A position error of 2.5 rotations results in ____ V output
		motorConfig.Slot0.kI = IntakeConstants.INTAKE_KI_0; // no output for integrated error
		motorConfig.Slot0.kD = IntakeConstants.INTAKE_KD_0; // A velocity error of 1 rps results in ____ V output
		motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
		// motorConfigurator.apply(slot0Configs);

		motorConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;
		motorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = IntakeConstants.GAIN_SCHEDULE_ERROR_THRESHOLD;

		motorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static; // Use cosine gravity compensation
		motorConfig.Slot1.kG = IntakeConstants.INTAKE_KG_1; // Gravity gain
		motorConfig.Slot1.kS = IntakeConstants.INTAKE_KS_1; // Add ____ V output to overcome static friction
		motorConfig.Slot1.kV = IntakeConstants.INTAKE_KV_1; // A velocity target of 1 rps results in ____ V output
		motorConfig.Slot1.kA = IntakeConstants.INTAKE_KA_1; // An acceleration of 1 rps/s requires ____ V output
		motorConfig.Slot1.kP = IntakeConstants.INTAKE_KP_1; // A position error of 2.5 rotations results in ____ V output
		motorConfig.Slot1.kI = IntakeConstants.INTAKE_KI_1; // no output for integrated error
		motorConfig.Slot1.kD = IntakeConstants.INTAKE_KD_1; // A velocity error of 1 rps results in ____ V output
		motorConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
		// motorConfigurator.apply(slot0Configs);_1
		motorConfig.Slot1.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;
		// MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
		// motionMagicConfigs.MotionMagicCruiseVelocity =
		// IntakeConstants.INTAKE_MM_CRUISE_VELOCITY;
		motorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.FAST_MAXIMUM_VELOCITY; // 80; // Target
																									// cruise
																									// velocity of
																									// 80 rps
		motorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.FAST_ACCELERATION; // 160; // Target
		// acceleration of
		// 160 rps/s (0.5
		// seconds)
		// motorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.INTAKE_MM_JERK; //
		// 1600; // Target jerk of 1600
		// rps/s/s (0.1 seconds)
		// motorConfigurator.apply(motionMagicConfigs);

		// feedback configs for to change gear ratio stuff
		// FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		// feedbackConfigs.RotorToSensorRatio = IntakeConstants.ROBOT_TO_SENSOR_RATIO;
		// motorConfig.Feedback.RotorToSensorRatio =
		// IntakeConstants.ROTOR_TO_SENSOR_RATIO; // no more more
		motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.SENSOR_TO_MECHANISM_RATIO;
		// motorConfigurator.apply(feedbackConfigs);

		for (int i = 0; i < 2; i++) {
			var status = motor.getConfigurator().apply(motorConfig);
			if (status.isOK()) {
				break;
			}
		}

		motor.setPosition(intakeEncoder.getAbsolutePosition().getValueAsDouble());

		setPointMeters = motor.getPosition().getValueAsDouble();

		// code to zero encoder when code is intially run (robot is turned on) so that
		// it doesn't freak out and try to spin to the zero from a previous continuous
		// motor run demo because the code of the shoulder has the motor spin to defined
		// finite values and the leftover value in the encoder will be huge and it will
		// zoom backward very powerfully. so it will not do that now as it takes a deep
		// breath before going anywhere.
		// TODO: hook up to absolute encoder on actual bot
		// Motor.setPosition(IntakeConstants.SHOULDER_STOWED_ANGLE);

		// Enable brake mode on robot enable
		RobotModeTriggers.disabled()
				.onFalse(enableBrakeModeCommand());

		// Set up temperature monitoring for the motors
		MotorMonitor.addMotor(motor);

		SmartDashboard.putData("Disable Intake Brake Mode", disableBrakeModeCommand());
	}

	/**
	 * Method which takes Angle values (no it does not) and sets Kraken to angle.
	 * Should compensate
	 * for gear ratios.
	 *
	 * @param positionMeters
	 */
	private void setPositionOutFast(double positionMeters) {
		positionMeters = MathUtil.clamp(positionMeters, IntakeConstants.SHOULDER_MINIMUM_DISTANCE, IntakeConstants.SHOULDER_MAXIMUM_DISTANCE);
		setPointMeters = positionMeters;
		// solution one (post *360) (doesn't work)
		// motor.setControl(positionRequest.withPosition(position.in(Units.Degrees)));
		motor.setControl(positionRequestOut.withPosition(positionMeters)); // max's solution

		// private void setPosition(double position) {
		// motor.setControl(positionRequest.withPosition(position));
	}

	/**
	 * Method which takes Angle values (no it does not) and sets Kraken to angle.
	 * Should compensate
	 * for gear ratios.
	 *
	 * @param positionMeters
	 */
	private void setPositionInSlow(double positionMeters) {
		positionMeters = MathUtil.clamp(positionMeters, IntakeConstants.SHOULDER_MINIMUM_DISTANCE, IntakeConstants.SHOULDER_MAXIMUM_DISTANCE);
		setPointMeters = positionMeters;
		// solution one (post *360) (doesn't work)
		// motor.setControl(positionRequest.withPosition(position.in(Units.Degrees)));
		motor.setControl(positionRequestIn.withPosition(positionMeters)); // max's solution

		// private void setPosition(double position) {
		// motor.setControl(positionRequest.withPosition(position));
	}

	// /**
	// * Command which raises shoulder of intake
	// *
	// * @return runOnce(() -> raiseIntake());
	// */
	// public Command raiseIntakeCommand() {
	// return runOnce(() -> raiseIntake());
	// }

	/**
	 * Command which raises shoulder of intake slowly
	 *
	 * @return runOnce(() -> raiseIntakeSlow());
	 */
	public Command raiseIntakeSlowCommand() {
		return runOnce(() -> raiseIntakeSlow());
	}

	/**
	 * Command which lowers shoulder of intake
	 *
	 * @return runOnce(() - > lowerIntake());
	 */
	public Command lowerIntakeCommand() {
		return runOnce(() -> lowerIntake());
	}

	/**
	 * Command which continously runs agitate method when triggered and raises
	 * intake when cancelled.
	 *
	 * @return run(() -> agitate()).finallyDo(() -> raiseIntake());
	 */
	// don't need this anymore now that we are a linear slide
	// public Command agitateCommand() {
	// return run(() -> agitate()).finallyDo(() -> raiseIntake());
	// }

	/**
	 * Command which continously runs agitate method when triggered and raises
	 * intake when cancelled.
	 *
	 * @return run(() -> agitate()).finallyDo(() -> raiseIntake());
	 */
	// don't need this anymore now that we are a linear slide
	public Command agitateCommand() {
		return lowerIntakeCommand().andThen(Commands.run(() -> agitate())).finallyDo(() -> stowOverBumper());
	}

	/**
	 * Command which calls lowerToDepot method.
	 *
	 * @return runOnce(() -> lowerToDepot());
	 */
	public Command lowerToDepotCommand() {
		return runOnce(() -> lowerToDepot());
	}

	/**
	 * Command which calls stowOverBumper method.
	 *
	 * @return runOnce(() -> stowOverBumper());
	 */
	public Command stowOverBumperCommand() {
		return runOnce(() -> stowOverBumper());
	}

	// /**
	// * Method which sets motor position to specified stowed angle.
	// */
	// private void raiseIntake() {
	// setPositionOutFast(IntakeConstants.SHOULDER_MINIMUM_DISTANCE);
	// // setPosition(0.25);
	// }

	/**
	 * Method which sets motor position to specified stowed angle slowly.
	 */
	private void raiseIntakeSlow() {
		setPositionInSlow(IntakeConstants.SHOULDER_MINIMUM_DISTANCE);
		// setPosition(0.25);
	}

	/**
	 * Method which sets motor position to specified lowered angle.
	 */
	private void lowerIntake() {
		setPositionOutFast(IntakeConstants.SHOULDER_MAXIMUM_DISTANCE);
		// setPosition(0);
	}

	/**
	 * Method which sets motor position to specified depot angle.
	 */
	private void lowerToDepot() {
		setPositionOutFast(IntakeConstants.SHOULDER_DEPOT_DISTANCE);
	}

	/**
	 * Method which sets motor position to specified stow over bumper angle.
	 */
	private void stowOverBumper() {
		setPositionInSlow(IntakeConstants.SHOULDER_STOW_OVER_BUMPER_DISTANCE);
	}

	public void zeroEncoder() {
		setPointMeters = intakeEncoder.getAbsolutePosition().getValueAsDouble();
		motor.setControl(positionRequestOut.withPosition(setPointMeters));
		motor.setPosition(intakeEncoder.getAbsolutePosition().getValueAsDouble());
	}

	/**
	 * Method which sets motor position to specified low angle. To be used in
	 * agitate command
	 */
	// private void lowerAgitate() {
	// setPositionPlease(IntakeConstants.AGITATE_LOWERED_ANGLE);
	// }

	/**
	 * Method which sets motor position to specified high angle. To be used in
	 * agitate command
	 */
	// private void raiseAgitate() {
	// setPositionPlease(IntakeConstants.AGITATE_RAISED_ANGLE);
	// }

	// TODO: utilize absolute encoder rather than internal encoder
	/**
	 * Method which checks angle of motor using Phoenix and WPILib commands.
	 *
	 * @return boolean indicating if motor position is within tolerance to angle
	 */
	public boolean getIsAtTarget() {
		return MathUtil.isNear(setPointMeters, motor.getPosition().getValueAsDouble(), IntakeConstants.SHOULDER_TOLERANCE);
	}

	/**
	 * Nudges the intake setpoint by a certain amount.
	 *
	 * @param nudgeAmount
	 *            The amount to nudge by.
	 * @return
	 *         Command to run.
	 */
	public Command nudgeIntakeCommand(Supplier<Double> nudgeAmount) {
		return runOnce(() -> {
			setPositionOutFast(setPointMeters += nudgeAmount.get() * IntakeConstants.NUDGE_SPEED);
		});
	}

	/**
	 * Runs getIsAtTarget() for specified raised angle (for agitate system)
	 *
	 * @return boolean value
	 */
	// private boolean getIsRaised() {
	// return getIsAtTarget(IntakeConstants.AGITATE_RAISED_ANGLE,
	// IntakeConstants.AGITATE_TOLERANCE);
	// }

	/**
	 * Runs getIsAtTarget() for specified lowered angle (for agitate system)
	 *
	 * @return boolean value
	 */
	// private boolean getIsLowered() {
	// return getIsAtTarget(IntakeConstants.AGITATE_LOWERED_ANGLE,
	// IntakeConstants.AGITATE_TOLERANCE);
	// }

	/**
	 * Method which moves arm to a raised position when in lowered position and vice
	 * versa. This is intended to be called through a continuous RunCommand
	 * triggered by a held button. Does not have an end state by default, this must
	 * be implemented in said Command.
	 */
	// depracated because of slide
	// private void agitate() {
	// if (getIsRaised()) {
	// lowerAgitate();
	// } else if (getIsLowered()) {
	// raiseAgitate();
	// }
	// }

	/**
	 * OOD
	 * Method which sets motor position to specified low angle. To be used in
	 * agitate command
	 */
	private void lowerAgitate() {
		setPositionInSlow(IntakeConstants.AGITATE_LOWERED_DISTANCE);
	}

	/**
	 * OOD
	 * Method which sets motor position to specified high angle. To be used in
	 * agitate command
	 */
	private void raiseAgitate() {
		setPositionInSlow(IntakeConstants.AGITATE_RAISED_DISTANCE);
	}

	public boolean getHasReachedTarget(double distance, double tolerance) {
		return MathUtil.isNear(distance, motor.getPosition().getValueAsDouble(), tolerance);
	}

	/**
	 * Runs getIsAtTarget() for specified raised angle (for agitate system)
	 *
	 * @return boolean value
	 */
	private boolean getIsRaised() {
		return getHasReachedTarget(IntakeConstants.AGITATE_RAISED_DISTANCE, IntakeConstants.AGITATE_TOLERANCE);
	}

	/**
	 * Runs getIsAtTarget() for specified lowered angle (for agitate system)
	 *
	 * @return boolean value
	 */
	private boolean getIsLowered() {
		return getHasReachedTarget(IntakeConstants.AGITATE_LOWERED_DISTANCE, IntakeConstants.AGITATE_TOLERANCE);
	}

	/**
	 * Method which moves arm to a raised position when in lowered position and vice
	 * versa. This is intended to be called through a continuous RunCommand
	 * triggered by a held button. Does not have an end state by default, this must
	 * be implemented in said Command.
	 */

	private void agitate() {
		if (getIsRaised()) {
			lowerAgitate();
		} else if (getIsLowered()) {
			raiseAgitate();
		}
	}

	/**
	 * Disables brake mode.
	 */
	public void disableBrakeMode() {
		motor.setNeutralMode(NeutralModeValue.Coast);
	}

	/**
	 * Enables brake mode.
	 */
	public void enableBrakeMode() {
		motor.setNeutralMode(NeutralModeValue.Brake);
	}

	/**
	 * Disables brake mode.
	 * <p>
	 * This command can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command disableBrakeModeCommand() {
		return Commands.runOnce(this::disableBrakeMode)
				.ignoringDisable(true);
	}

	/**
	 * Enables brake mode.
	 * <p>
	 * This command can be run while disabled.
	 *
	 * @return
	 *         Command to run.
	 * @implNote
	 *           Called internally on enable to reset things.
	 */
	public Command enableBrakeModeCommand() {
		return Commands.runOnce(this::enableBrakeMode)
				.ignoringDisable(true);
	}
}
