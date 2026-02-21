package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class IntakeShoulder extends SubsystemBase {
	// controller for motor which moves intake
	private final TalonFX motor;

	/**
	 * Constructor for motor which raises and lowers intakes with associated configurations, sets up Motion Magic, and configures gear ratios.
	 */
	public IntakeShoulder() {
		motor = new TalonFX(IntakeConstants.SHOULDER_CAN_ID);

		TalonFXConfigurator motorConfigurator = motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLowerLimit = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LOWER_LIMIT;
		limitConfigs.SupplyCurrentLimit = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLowerTime = IntakeConstants.SHOULDER_SUPPLY_CURRENT_LOWER_TIME;
		limitConfigs.SupplyCurrentLimitEnable = true;
		limitConfigs.StatorCurrentLimit = IntakeConstants.SHOULDER_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;
		motorConfigurator.apply(limitConfigs);

		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		// Puts the motor in Coast mode to make it easier to move by hand
		motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
		// Configure the motor to make sure positive voltage is counter clockwise
		motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
		motorConfigurator.apply(motorOutputConfigs);

		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; // Use cosine gravity compensation
		slot0Configs.kG = IntakeConstants.INTAKE_KG; // Gravity gain
		slot0Configs.kS = IntakeConstants.INTAKE_KS; // Add ____ V output to overcome static friction
		slot0Configs.kV = IntakeConstants.INTAKE_KV; // A velocity target of 1 rps results in ____ V output
		slot0Configs.kA = IntakeConstants.INTAKE_KA; // An acceleration of 1 rps/s requires ____ V output
		slot0Configs.kP = IntakeConstants.INTAKE_KP; // A position error of 2.5 rotations results in ____ V output
		slot0Configs.kI = IntakeConstants.INTAKE_KI; // no output for integrated error
		slot0Configs.kD = IntakeConstants.INTAKE_KD; // A velocity error of 1 rps results in ____ V output
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
		motorConfigurator.apply(slot0Configs);

		MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
		motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_MM_CRUISE_VELOCITY; // 80; // Target cruise velocity of 80 rps
		motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.INTAKE_MM_ACCELERATION; // 160; // Target acceleration of 160 rps/s (0.5 seconds)
		motionMagicConfigs.MotionMagicJerk = IntakeConstants.INTAKE_MM_JERK; // 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
		motorConfigurator.apply(motionMagicConfigs);

		// feedback configs for to change gear ratio stuff
		FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		feedbackConfigs.RotorToSensorRatio = IntakeConstants.ROBOT_TO_SENSOR_RATIO;
		feedbackConfigs.SensorToMechanismRatio = IntakeConstants.SENSOR_TO_MECHANISM_RATIO;
		motorConfigurator.apply(feedbackConfigs);

		// code to zero encoder when code is intially run (robot is turned on) so that it doesn't freak out and try to spin to the zero from a previous continuous motor run demo because the code of the shoulder has the motor spin to defined finite values and the leftover value in the encoder will be huge and it will zoom backward very powerfully. so it will not do that now as it takes a deep breath before going anywhere.
		// TODO: hook up to absolute encoder on actual bot
		motor.setPosition(IntakeConstants.SHOULDER_STOWED_ANGLE);
	}

	private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

	/**
	 * Method which takes Angle values and sets Kraken to angle. Should compensate for gear ratios.
	 * 
	 * @param position
	 */
	private void setPositionPlease(Angle position) {
		// solution one (post *360) (doesn't work)
		// motor.setControl(positionRequest.withPosition(position.in(Units.Degrees)));
		motor.setControl(positionRequest.withPosition(position)); // max's possible solution (seems to work)

		// private void setPosition(double position) {
		// motor.setControl(positionRequest.withPosition(position));
	}

	/**
	 * Command which raises shoulder of intake
	 * 
	 * @return runOnce(() -> raiseIntake());
	 */
	public Command raiseIntakeCommand() {
		return runOnce(() -> raiseIntake());
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
	 * Command which continously runs agitate method when triggered and raises intake when cancelled.
	 * 
	 * @return run(() -> agitate()).finallyDo(() -> raiseIntake());
	 */
	public Command agitateCommand() {
		return run(() -> agitate()).finallyDo(() -> raiseIntake());
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
	 * Method which sets motor position to specified stowed angle.
	 */
	public void raiseIntake() {
		setPositionPlease(IntakeConstants.SHOULDER_STOWED_ANGLE);
		// setPosition(0.25);
	}

	/**
	 * Method which sets motor position to specified lowered angle.
	 */
	public void lowerIntake() {
		setPositionPlease(IntakeConstants.SHOULDER_LOWERED_ANGLE);
		// setPosition(0);
	}

	/**
	 * Method which sets motor position to specified depot angle.
	 */
	public void lowerToDepot() {
		setPositionPlease(IntakeConstants.SHOULDER_DEPOT_ANGLE);
	}

	/**
	 * Method which sets motor position to specified low angle. To be used in agitate command
	 */
	public void lowerAgitate() {
		setPositionPlease(IntakeConstants.AGITATE_LOWERED_ANGLE);
	}

	/**
	 * Method which sets motor position to specified high angle. To be used in agitate command
	 */
	public void raiseAgitate() {
		setPositionPlease(IntakeConstants.AGITATE_RAISED_ANGLE);
	}

	// TODO: utilize absolute encoder rather than internal encoder
	/**
	 * Method which checks angle of motor using Phoenix and WPILib commands.
	 * 
	 * @param angle
	 *            - angle to be chcked as Angle
	 * @param tolerance
	 *            - acceptable range as Angle
	 * @return boolean indicating if motor position is within tolerance to angle
	 */
	private boolean getIsAtTarget(Angle angle, Angle tolerance) {
		return motor.getPosition()
				.getValue()
				.isNear(angle, tolerance);
	}

	/**
	 * Runs getIsAtTarget() for specified raised angle (for agitate system)
	 * 
	 * @return boolean value
	 */
	private boolean getIsRaised() {
		return getIsAtTarget(IntakeConstants.AGITATE_RAISED_ANGLE, IntakeConstants.AGITATE_TOLERANCE);
	}

	/**
	 * Runs getIsAtTarget() for specified lowered angle (for agitate system)
	 * 
	 * @return boolean value
	 */
	private boolean getIsLowered() {
		return getIsAtTarget(IntakeConstants.AGITATE_LOWERED_ANGLE, IntakeConstants.AGITATE_TOLERANCE);
	}

	/**
	 * Method which moves arm to a raised position when in lowered position and vice versa. This is intended to be called through a continuous RunCommand triggered by a held button. Does not have an end state by default, this must be implemented in said Command.
	 */
	public void agitate() {
		// this if system is downright dubious
		if (getIsRaised()) {
			lowerAgitate();
		} else if (getIsLowered()) {
			raiseAgitate();
		}
	}
}
