package frc.robot.subsystems;

import java.lang.management.ClassLoadingMXBean;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
	private final TalonFX CIBMotor;

	private final MotionMagicVoltage CIBPositionOut = new MotionMagicVoltage(0);

	public Climber() {
		CIBMotor = new TalonFX(ClimbConstants.CIB_MOTOR_CAN_ID);

		TalonFXConfiguration CIBConfig = new TalonFXConfiguration();
		CIBConfig.Slot0.kS = ClimbConstants.CIB_KS;
		CIBConfig.Slot0.kV = ClimbConstants.CIB_KV;
		CIBConfig.Slot0.kP = ClimbConstants.CIB_KP;
		CIBConfig.Slot0.kP = ClimbConstants.CIB_KI;
		CIBConfig.Slot0.kP = ClimbConstants.CIB_KD;
		CIBConfig.Slot0.kG = ClimbConstants.CIB_KG;
		CIBConfig.Slot0.kA = ClimbConstants.CIB_KA;
		CIBConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		CIBConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY;
		CIBConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION;

		StatusCode status;
		for (int i = 0; i < 2; ++i) {
			status = CIBMotor.getConfigurator().apply(CIBConfig);
			if (status.isOK()) {
				break;
			}
		}
	}

	/*
	 * moves the CIB's position to the specified position in meters
	 */
	private void moveTo(double position) {
		CIBMotor.setControl(CIBPositionOut.withPosition(metersToRotations(position + ClimbConstants.CIB_MIN_POSITION)));
	}

	/*
	 * converts the CIB's position in rotations of the motor to meters,
	 * may not be perfectly accurate at positions between the min and max due to the rope spooling on top of itself
	 */
	private double rotationsToMeters(double rotations) {
		return ((rotations * ClimbConstants.CIB_GEARBOX_RATIO) / (ClimbConstants.CIB_MAX_ROTATIONS - ClimbConstants.CIB_MIN_ROTATIONS)) * (ClimbConstants.CIB_MAX_POSITION - ClimbConstants.CIB_MIN_POSITION);
	}

	/*
	 * converts the CIB's position in meters to rotations of the motor,
	 * may not be perfectly accurate at positions between the min and max due to the rope spooling on top of itself
	 */
	private double metersToRotations(double meters) {
		return ((meters / (ClimbConstants.CIB_MAX_POSITION - ClimbConstants.CIB_MIN_POSITION)) * (ClimbConstants.CIB_MAX_ROTATIONS - ClimbConstants.CIB_MIN_ROTATIONS)) / ClimbConstants.CIB_GEARBOX_RATIO;
	}

	/*
	 * returns the climbers position in meters
	 */
	private double getPositionMeters() {
		return rotationsToMeters(getPositionRotations());
	}

	/*
	 * returns the climbers position in rotations
	 */
	private double getPositionRotations() {
		return CIBMotor.getPosition().getValueAsDouble() - ClimbConstants.CIB_MIN_ROTATIONS;
	}

	private void raiseClimb() {
		if (getPositionMeters() > ClimbConstants.CIB_MIN_POSITION + ClimbConstants.TOLERANCE) {
			System.out.println("WARNING: Elevator not fully lowered before deploying climb");
		}
		moveTo(ClimbConstants.CIB_MAX_POSITION);
	}

	private void lowerClimb() {
		if (getPositionMeters() < ClimbConstants.CIB_MAX_POSITION - ClimbConstants.TOLERANCE) {
			System.out.println("WARNING: Elevator not fully deployed before lowering climb");
		}
		moveTo(ClimbConstants.CIB_MIN_POSITION);
	}

	public Command getRaiseClimbCommand() {
		return run(() -> raiseClimb());
	}

	public Command getLowerClimbCommand() {
		return run(() -> lowerClimb());
	}

	@Override
	public void periodic() {}
}
