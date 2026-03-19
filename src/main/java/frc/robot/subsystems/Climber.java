package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
	private final TalonFX climberMotor;

	private final MotionMagicVoltage climberPositionOut = new MotionMagicVoltage(0);
	private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

	public Climber() {
		climberMotor = new TalonFX(ClimbConstants.CIB_MOTOR_CAN_ID);

		TalonFXConfiguration climberConfig = new TalonFXConfiguration();
		climberConfig.Slot0.kS = ClimbConstants.CIB_KS;
		climberConfig.Slot0.kV = ClimbConstants.CIB_KV;
		climberConfig.Slot0.kP = ClimbConstants.CIB_KP;
		climberConfig.Slot0.kP = ClimbConstants.CIB_KI;
		climberConfig.Slot0.kP = ClimbConstants.CIB_KD;
		climberConfig.Slot0.kG = ClimbConstants.CIB_KG;
		climberConfig.Slot0.kA = ClimbConstants.CIB_KA;

		climberConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
		climberConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		climberConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY;
		climberConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION;

		climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		climberConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.CIB_STATOR_CURRENT_LIMIT;
		climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		climberConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.CIB_SUPPLY_CURRENT_LIMIT;
		climberConfig.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.CIB_SUPPLY_CURRENT_LOWER_LIMIT;
		climberConfig.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.CIB_SUPPLY_CURRENT_LIMIT_LOWER_TIME;
		climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		StatusCode status;
		for (int i = 0; i < 3; i++) {
			status = climberMotor.getConfigurator().apply(climberConfig);
			if (status.isOK()) {
				break;
			}
		}

		climberMotor.setPosition(0);
	}

	/*
	 * moves the CIB's position to the specified position in meters
	 */
	private void moveToMeters(double position) {
		climberMotor.setControl(climberPositionOut.withPosition(metersToRotations(position)));
	}

	private void goSpeed(Double speed) {
		climberMotor.setControl(dutyCycleOut.withOutput(speed));
	}

	/*
	 * moves the CIB's position to the specified position in rotations
	 */
	private void moveToRotations(double rotations) {
		climberMotor.setControl(climberPositionOut.withPosition(rotations));
	}

	/*
	 * converts the CIB's position in rotations of the motor to meters,
	 * may not be perfectly accurate at positions between the min and max due to the rope spooling on top of itself
	 */
	private double rotationsToMeters(double rotations) {
		return ((rotations * ClimbConstants.CIB_GEARBOX_RATIO) / (ClimbConstants.CIB_MAX_ROTATIONS)) * (ClimbConstants.CIB_MAX_POSITION);
	}

	/*
	 * converts the CIB's position in meters to rotations of the motor,
	 * may not be perfectly accurate at positions between the min and max due to the rope spooling on top of itself
	 */
	private double metersToRotations(double meters) {
		return ((meters / (ClimbConstants.CIB_MAX_POSITION)) * (ClimbConstants.CIB_MAX_ROTATIONS)) / ClimbConstants.CIB_GEARBOX_RATIO;
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
		return climberMotor.getPosition().getValueAsDouble();
	}

	private void raiseClimb() {
		if (getPositionMeters() > ClimbConstants.TOLERANCE) {
			System.out.println("WARNING: Elevator not fully lowered before deploying climb");
		}
		moveToRotations(ClimbConstants.CIB_MAX_ROTATIONS);
		// moveToMeters(ClimbConstants.CIB_MAX_POSITION);
	}

	private void lowerClimb() {
		if (getPositionMeters() < ClimbConstants.CIB_MAX_POSITION - ClimbConstants.TOLERANCE) {
			System.out.println("WARNING: Elevator not fully deployed before lowering climb");
		}
		moveToRotations(0);
		// moveToMeters(0);
	}

	public boolean isAtTarget() {
		return MathUtil.isNear(climberPositionOut.Position, getPositionRotations(), ClimbConstants.TOLERANCE);
	}

	public Command RaiseClimbCommand() {
		return runOnce(() -> raiseClimb());
	}

	public Command LowerClimbCommand() {
		return runOnce(() -> lowerClimb());
	}

	public Command moveUpManuallyCommand() {
		return run(() -> goSpeed(ClimbConstants.UP_SPEED)).finallyDo(() -> goSpeed(0.0));
	}

	public Command moveDownManuallyCommand() {
		return run(() -> goSpeed(ClimbConstants.DOWN_SPEED)).finallyDo(() -> goSpeed(0.0));
	}

	@Override
	public void periodic() {}
}
