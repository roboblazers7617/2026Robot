package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeGrabber extends SubsystemBase {
	// controller for motor which intakes
	private final TalonFX Motor;

	public IntakeGrabber() {
		Motor = new TalonFX(IntakeConstants.GRABBER_CAN_ID);

		TalonFXConfigurator MotorConfigurator = Motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLowerLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_LIMIT;
		limitConfigs.SupplyCurrentLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLowerTime = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_TIME;
		limitConfigs.SupplyCurrentLimitEnable = true;
		limitConfigs.StatorCurrentLimit = IntakeConstants.GRABBER_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;
		MotorConfigurator.apply(limitConfigs);

		MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
		outputConfigs.NeutralMode = NeutralModeValue.Coast;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
		MotorConfigurator.apply(outputConfigs);
	}

	private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
	// Motor.setControl(torqueCurrent.withOutput(40.0));

	/**
	 * command which toggles intake on
	 * 
	 * @param speed
	 */
	public Command startIntakeCommand() {
		return runOnce(() -> startIntake());
	}

	/**
	 * command which toggles intake off
	 */
	public Command stopIntakeCommand() {
		return runOnce(() -> stopIntake());
	}

	public Command outtakeCommand() {
		return runOnce(() -> outtake());
	}

	public void startIntake() {
		// setSpeed(IntakeConstants.INTAKE_START_SPEED);
		setTorque(IntakeConstants.INTAKE_START_TORQUE);
	}

	public void stopIntake() {
		// setSpeed(IntakeConstants.INTAKE_STOP_SPEED);
		setTorque(IntakeConstants.INTAKE_STOP_TORQUE);
	}

	public void outtake() {
		// setSpeed(IntakeConstants.OUTTAKE_SPEED);
		setTorque(IntakeConstants.INTAKE_STOP_TORQUE);
	}

	/**
	 * method to make intake wheels go at
	 * 
	 * @param speed
	 *            Speed [-1,1].
	 */
	// private void setSpeed(double speed) {
	// Motor.set(speed);
	// }

	private void setTorque(double torque) {
		Motor.setControl(torqueCurrent.withOutput(torque));
	}
}
