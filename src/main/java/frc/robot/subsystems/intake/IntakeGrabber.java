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

	/**
	 * Constructor for motor for actual intaking part of intake. Uses
	 * TorqueCurrentFOC.
	 */
	public IntakeGrabber() {
		Motor = new TalonFX(IntakeConstants.GRABBER_CAN_ID);

		TalonFXConfigurator MotorConfigurator = Motor.getConfigurator();

		// Current limit configuration
		// TODO: You don't need to create new objects for CurrentLimitConfigs,
		// MotorOutputConfigs, etc.
		// You can just do the following...
		// var talonFXConfigs = new TalonFXConfiguration();
		// var limitConfigs = talonFXConfigs.limitConfigs;
		// etc.
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLowerLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_LIMIT;
		limitConfigs.SupplyCurrentLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLowerTime = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_TIME;
		limitConfigs.SupplyCurrentLimitEnable = true;
		limitConfigs.StatorCurrentLimit = IntakeConstants.GRABBER_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;
		// TODO: Don't need to apply these configs if you use the code above. Just need
		// to apply once
		MotorConfigurator.apply(limitConfigs);

		MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
		outputConfigs.NeutralMode = NeutralModeValue.Coast;
		outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
		// TODO: Don't need to apply these configs individually if you use the code
		// above.
		MotorConfigurator.apply(outputConfigs);
		// TODO: (Benjamin) The apply for the configs should be wrapped in a for loop to
		// ensure it works. See the sample code in the document I created about
		// configuring the talons
	}

	// TODO: This should be at the top of the class before the constructor
	private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
	// Motor.setControl(torqueCurrent.withOutput(40.0));

	/**
	 * Command which toggles intake on
	 * 
	 * @return runOnce(() -> startIntake());
	 */
	public Command startIntakeCommand() {
		return runOnce(() -> startIntake());
	}

	/**
	 * Command which toggles intake off
	 * 
	 * @return runOnce(() -> stopIntake());
	 */
	public Command stopIntakeCommand() {
		return runOnce(() -> stopIntake());
	}

	/**
	 * Command which initiates outtake, to be triggered with held button. Stops
	 * intake when released.
	 * 
	 * @return run(() -> outtake()).finallyDo(() -> stopIntake());
	 */
	public Command outtakeCommand() {
		return run(() -> outtake()).finallyDo(() -> stopIntake());
	}

	/**
	 * Method which starts the intake with a TorqueCurrentFOC method.
	 */
	private void startIntake() {
		// setSpeed(IntakeConstants.INTAKE_START_SPEED);
		setTorque(IntakeConstants.INTAKE_START_CURRENT);
	}

	/**
	 * Method which stops the intake with a TorqueCurrentFOC method.
	 */
	private void stopIntake() {
		// setSpeed(IntakeConstants.INTAKE_STOP_SPEED);
		setTorque(IntakeConstants.INTAKE_STOP_CURRENT);
	}

	/**
	 * Method which starts the intake in reverse with a TorqueCurrentFOC method.
	 */
	private void outtake() {
		// setSpeed(IntakeConstants.OUTTAKE_SPEED);
		setTorque(IntakeConstants.OUTTAKE_CURRENT);
	}

	/**
	 * Method which makes intake motors spin at a certain speed utilizing
	 * TorqueCurrentFOC.
	 * 
	 * @param current
	 *                [-1,1].
	 */
	// private void setSpeed(double speed) {
	// Motor.set(speed);
	// }
	// TODO: This isn't a torque. It is a current
	// okay
	private void setTorque(double current) {
		Motor.setControl(torqueCurrent.withOutput(current));
	}
}
