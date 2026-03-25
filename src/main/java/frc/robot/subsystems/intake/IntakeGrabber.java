package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeGrabber extends SubsystemBase {
	// controller for motor which intakes
	private final TalonFX Motor;

	private final VoltageOut voltageOut = new VoltageOut(0);
	// private final TorqueCurrentFOC torqueCurrent = new TorqueCurrentFOC(0);
	// Motor.setControl(torqueCurrent.withOutput(40.0));

	/**
	 * Constructor for motor for actual intaking part of intake. Uses
	 * TorqueCurrentFOC.
	 */
	public IntakeGrabber() {
		Motor = new TalonFX(IntakeConstants.GRABBER_CAN_ID, Constants.CANIVORE_BUS);

		// TalonFXConfigurator MotorConfigurator = Motor.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();

		// Current limit configuration
		// TODO: You don't need to create new objects for CurrentLimitConfigs,
		// MotorOutputConfigs, etc.
		// You can just do the following...
		// var talonFXConfigs = new TalonFXConfiguration();
		// var limitConfigs = talonFXConfigs.limitConfigs;
		// etc.

		// CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		// limitConfigs.SupplyCurrentLowerLimit =
		// IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.GRABBER_SUPPLY_CURRENT_LIMIT;
		motorConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.GRABBER_SUPPLY_CURRENT_LOWER_TIME;
		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.GRABBER_STATOR_CURRENT_LIMIT;
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		// TODO: Don't need to apply these configs if you use the code above. Just need

		// MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		// TODO: Don't need to apply these configs individually if you use the code
		// above.
		// MotorConfigurator.apply(outputConfigs);
		// TODO: (Benjamin) The apply for the configs should be wrapped in a for loop to
		// ensure it works. See the sample code in the document I created about
		// configuring the talons

		for (int i = 0; i < 2; i++) {
			var status = Motor.getConfigurator().apply(motorConfig);
			if (status.isOK()) {
				break;
			}
		}
	}

	/**
	 * Command which toggles intake on
	 *
	 * @return runOnce(() -> startIntake());
	 */
	public Command startIntakeCommand() {
		return runOnce(() -> startIntake());
	}

	/**
	 * Command which toggles intake on at a slow speed
	 *
	 * @return runOnce(() -> startIntakeSlow());
	 */
	public Command startIntakeSlowCommand() {
		return runOnce(() -> startIntakeSlow());
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
		return runOnce(() -> outtake());
	}

	/**
	 * Method which starts the intake with a TorqueCurrentFOC method.
	 */
	private void startIntake() {
		// setSpeed(IntakeConstants.INTAKE_START_SPEED);
		setVoltage(IntakeConstants.INTAKE_START_VOLTAGE);
	}

	/**
	 * Method which starts the intake with a TorqueCurrentFOC method at a slower speed.
	 */
	private void startIntakeSlow() {
		// setSpeed(IntakeConstants.INTAKE_START_SPEED);
		setVoltage(IntakeConstants.INTAKE_START_SLOW_VOLTAGE);
	}

	/**
	 * Method which stops the intake with a TorqueCurrentFOC method.
	 */
	public void stopIntake() {
		// setSpeed(IntakeConstants.INTAKE_STOP_SPEED);
		setVoltage(IntakeConstants.INTAKE_STOP_VOLTAGE);
	}

	/**
	 * Method which starts the intake in reverse with a TorqueCurrentFOC method.
	 */
	private void outtake() {
		// setSpeed(IntakeConstants.OUTTAKE_SPEED);
		setVoltage(IntakeConstants.OUTTAKE_VOLTAGE);
	}

	/**
	 * Method which makes intake motors spin at a certain speed
	 *
	 * @param voltage
	 *            [-1,1].
	 */
	// private void setSpeed(double speed) {
	// Motor.set(speed);
	// }
	private void setVoltage(double voltage) {
		Motor.setControl(voltageOut.withOutput(voltage));
	}
}
