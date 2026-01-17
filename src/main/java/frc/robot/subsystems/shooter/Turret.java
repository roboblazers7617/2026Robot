package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * The turret that the shooter is attached to.
 */
public class Turret extends SubsystemBase {
	private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID);

	private final PositionVoltage positionRequest = new PositionVoltage(0)
			.withSlot(0);

	public Turret() {
		TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();

		// Current limit configuration
		CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
		limitConfigs.SupplyCurrentLimit = TurretConstants.MOTOR_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;
		talonFXConfigurator.apply(limitConfigs);

		// PID configuration
		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kP = TurretConstants.TURRET_KP;
		slot0Configs.kI = TurretConstants.TURRET_KI;
		slot0Configs.kD = TurretConstants.TURRET_KD;
		talonFXConfigurator.apply(slot0Configs);
	}

	/**
	 * Commands the turret to a certain position.
	 *
	 * @param position
	 *            Position in rotations.
	 */
	public void setPosition(double position) {
		motor.setControl(positionRequest.withPosition(position));
	}

	/**
	 * Command to set the turret to a certain position
	 *
	 * @param position
	 *            Position in rotations.
	 */
	public Command setPositionCommand(Supplier<Double> position) {
		return runOnce(() -> setPosition(position.get()));
	}
}
