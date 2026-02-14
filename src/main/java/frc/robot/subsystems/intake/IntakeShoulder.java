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

		/**
		 * code to zero encoder when code is intially run (robot is turned on) so that it doesn't freak out and try to spin to the zero from a previous continuous motor run demo because the code of the shoulder has the motor spin to defined finite values and the leftover value in the encoder will be huge and it will zoom backward very powerfully. so it will not do that now as it takes a deep breath before going anywhere.
		 */

		// TODO: hook up to absolute encoder on actual bot
		motor.setPosition(IntakeConstants.SHOULDER_STOWED_ANGLE);
	}

	private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

	private void setPositionPlease(Angle position) {
		// solution one (post *360) (doesn't work)
		// motor.setControl(positionRequest.withPosition(position.in(Units.Degrees)));
		motor.setControl(positionRequest.withPosition(position)); // max's possible solution (seems to work)

		// private void setPosition(double position) {
		// motor.setControl(positionRequest.withPosition(position));
	}

	/**
	 * command which raises shoulder of intake
	 * 
	 * @return
	 */
	public Command raiseIntakeCommand() {
		return runOnce(() -> raiseIntake());
	}

	/**
	 * command which lowers shoulder of intake
	 * 
	 * @return
	 */
	public Command lowerIntakeCommand() {
		return runOnce(() -> lowerIntake());
	}

	public Command agitateCommand() {
		return run(() -> agitate()).finallyDo(() -> raiseIntake());
	}

	public void raiseIntake() {
		setPositionPlease(IntakeConstants.SHOULDER_STOWED_ANGLE);
		// setPosition(0.25);
	}

	public void lowerIntake() {
		setPositionPlease(IntakeConstants.SHOULDER_LOWERED_ANGLE);
		// setPosition(0);
	}

	public void lowerAgitate() {
		setPositionPlease(IntakeConstants.AGITATE_LOWERED_ANGLE);
	}

	public void raiseAgitate() {
		setPositionPlease(IntakeConstants.AGITATE_RAISED_ANGLE);
	}

	private boolean getIsAtTarget(Angle angle, Angle tolerance) {
		return motor.getPosition()
				.getValue()
				.isNear(angle, tolerance);
	}

	private boolean getIsRaised() {
		return getIsAtTarget(IntakeConstants.AGITATE_RAISED_ANGLE, IntakeConstants.AGITATE_TOLERANCE);
	}

	private boolean getIsLowered() {
		return getIsAtTarget(IntakeConstants.AGITATE_LOWERED_ANGLE, IntakeConstants.AGITATE_TOLERANCE);
	}

	// untested method which will continuously move the arm up and down when button is held then ask john about if up or down after button released
	// update I think I have it figured out must be tested now
	public void agitate() {
		// this if system is downright dubious
		if (getIsRaised()) {
			lowerAgitate();
		} else if (getIsLowered()) {
			raiseAgitate();
		}
	}
}
