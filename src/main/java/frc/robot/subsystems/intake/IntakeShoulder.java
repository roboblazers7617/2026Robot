package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
		limitConfigs.SupplyCurrentLimit = IntakeConstants.MOTOR_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;
		motorConfigurator.apply(limitConfigs);

		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		// Put's the motor in Coast mode to make it easier to move by hand
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

		/**
		 * code to zero the encoder (so it doesn't freak out and go to what it previously decided was zero from a continous motor demo because the code I did here was based on specific finite angles which are relative to the big spinny tests)
		 */

		motor.setPosition(0);
	}

	private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

	private void setPositionPlease(Angle position) {
		double positionDouble = position.in(Units.Degrees) / (360.0 * IntakeConstants.GEARBOX_RATIO);
		motor.setControl(positionRequest.withPosition(positionDouble));
	}

	// private void setPosition(double position) {
	// motor.setControl(positionRequest.withPosition(position));
	// }

	/**
	 * command which raises shoulder of intake
	 * 
	 * @return
	 */
	public Command raiseShoulderCommand() {
		return runOnce(() -> raiseShoulder());
	}

	/**
	 * command which lowers shoulder of intake
	 * 
	 * @return
	 */
	public Command lowerShoulderCommand() {
		return runOnce(() -> lowerShoulder());
	}

	public void raiseShoulder() {
		setPositionPlease(IntakeConstants.SHOULDER_RAISED_ANGLE);
		// setPosition(0.25);
	}

	public void lowerShoulder() {
		setPositionPlease(IntakeConstants.SHOULDER_LOWERED_ANGLE);
		// setPosition(0);
	}
}
