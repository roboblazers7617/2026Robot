package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeGrabber extends SubsystemBase {
	// controller for motor which intakes
	private final TalonFX Motor;

	public IntakeGrabber() {
		Motor = new TalonFX(IntakeConstants.GRABBER_CAN_ID);

		TalonFXConfigurator leaderMotorConfigurator = Motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = IntakeConstants.MOTOR_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;
		leaderMotorConfigurator.apply(limitConfigs);
	}

	/**
	 * command which toggles intake on
	 * 
	 * @param speed
	 * @return
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

	public void startIntake() {
		setSpeed(IntakeConstants.INTAKE_START_SPEED);
	}

	public void stopIntake() {
		setSpeed(IntakeConstants.INTAKE_STOP_SPEED);
	}

	/**
	 * method to make intake wheels go at
	 * 
	 * @param speed
	 *            Speed [-1,1].
	 */
	private void setSpeed(double speed) {
		Motor.set(speed);
	}
}
