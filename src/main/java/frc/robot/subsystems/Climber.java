package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
	private final TalonFX elevatorLeaderMotor;
	private final TalonFX followerElevatorMotor;

	private final TalonFX topClimbHookMotor;
	private final TalonFX bottomClimbHookMotor;

	private final CANcoder topHookEncoder;
	private final CANcoder bottomHookEncoder;

	private final MotionMagicVoltage elevatorPositionOut = new MotionMagicVoltage(0);
	private final MotionMagicVoltage topHookPositionOut = new MotionMagicVoltage(0);
	private final MotionMagicVoltage bottomHookPositionOut = new MotionMagicVoltage(0);

	public Climber() {
		elevatorLeaderMotor = new TalonFX(ClimbConstants.ELEVATOR_LEADER_MOTOR_CAN_ID);
		followerElevatorMotor = new TalonFX(ClimbConstants.ELEVATOR_FOLLOWER_MOTOR_CAN_ID);
		topClimbHookMotor = new TalonFX(ClimbConstants.TOP_HOOK_MOTOR_CAN_ID);
		bottomClimbHookMotor = new TalonFX(ClimbConstants.BOTTOM_HOOK_MOTOR_CAN_ID);

		TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
		elevatorConfig.Slot0.kS = ClimbConstants.ELEVATOR_KS;
		elevatorConfig.Slot0.kV = ClimbConstants.ELEVATOR_KV;
		elevatorConfig.Slot0.kP = ClimbConstants.ELEVATOR_KP;
		elevatorConfig.Slot0.kP = ClimbConstants.ELEVATOR_KI;
		elevatorConfig.Slot0.kP = ClimbConstants.ELEVATOR_KD;
		elevatorConfig.Slot0.kG = ClimbConstants.ELEVATOR_KG;
		elevatorConfig.Slot0.kA = ClimbConstants.ELEVATOR_KA;
		elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

		elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY;
		elevatorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION;

		TalonFXConfiguration topHookMotorConfig = new TalonFXConfiguration();
		topHookMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		topHookMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MAX_HOOK_VELOCITY;
		topHookMotorConfig.MotionMagic.MotionMagicAcceleration = ClimbConstants.MAX_HOOK_ACCELERATION;
		topHookMotorConfig.Slot0.kS = ClimbConstants.HOOK_KS;
		topHookMotorConfig.Slot0.kV = ClimbConstants.HOOK_KV;
		topHookMotorConfig.Slot0.kP = ClimbConstants.HOOK_KP;
		topHookMotorConfig.Slot0.kP = ClimbConstants.HOOK_KI;
		topHookMotorConfig.Slot0.kP = ClimbConstants.HOOK_KD;
		topHookMotorConfig.Slot0.kG = ClimbConstants.HOOK_KG;
		topHookMotorConfig.Slot0.kA = ClimbConstants.HOOK_KA;
		topHookMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		TalonFXConfiguration bottomHookMotorConfig = topHookMotorConfig.clone();
		// TODO update when we know what encoders we will be using
		topHookEncoder = new CANcoder(ClimbConstants.TOP_HOOK_ENCODER_CAN_ID);
		bottomHookEncoder = new CANcoder(ClimbConstants.BOTTOM_HOOK_ENCODER_CAN_ID);

		topHookMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		topHookMotorConfig.Feedback.FeedbackRemoteSensorID = topHookEncoder.getDeviceID();

		bottomHookMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		bottomHookMotorConfig.Feedback.FeedbackRemoteSensorID = bottomHookEncoder.getDeviceID();

		/*
		 * Apply configs multiple times bc Caleb says it breaks sometimes
		 */
		StatusCode status;
		for (int i = 0; i < 2; ++i) {
			status = elevatorLeaderMotor.getConfigurator().apply(elevatorConfig);
			if (status.isOK()) {
				break;
			}
		}
		for (int i = 0; i < 2; ++i) {
			status = followerElevatorMotor.getConfigurator().apply(elevatorConfig);
			if (status.isOK()) {
				followerElevatorMotor.setControl(new Follower(elevatorLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
				break;
			}
		}
		for (int i = 0; i < 2; ++i) {
			status = topClimbHookMotor.getConfigurator().apply(topHookMotorConfig);
			if (status.isOK()) {
				break;
			}
		}
		for (int i = 0; i < 2; ++i) {
			status = bottomClimbHookMotor.getConfigurator().apply(bottomHookMotorConfig);
			if (status.isOK()) {
				break;
			}
		}
	}

	/*
	 * pos
	 */
	private void moveTo(double position) {
		if (position > ClimbConstants.ELEVATOR_MAX_POSITION) {
			throw new Error("position given " + position + " is more than max elevator height of " + ClimbConstants.ELEVATOR_MAX_POSITION);
		} else if (position < ClimbConstants.ELEVATOR_MIN_POSITION) {
			throw new Error("position given " + position + " is less than min elevator height of " + ClimbConstants.ELEVATOR_MIN_POSITION);
		}
		elevatorLeaderMotor.setControl(elevatorPositionOut.withPosition(metersToRotations(position)));
	}

	/*
	 * converts vertical position of the elevator in meters to rotations of the motor
	 */
	private double metersToRotations(double meters) {
		return meters / (Units.inchesToMeters(ClimbConstants.ELEVATOR_CHAIN_IN_PER_TOOTH) * ClimbConstants.ELEVATOR_CHAIN_SPROCKET_TEETH) * ClimbConstants.ELEVATOR_GEARBOX_RATIO;
	}

	/*
	 * converts rotations of the motor to vertical position of the elevator in inches
	 */
	private double rotationsToMeters(double rotations) {
		return rotations * Units.inchesToMeters(ClimbConstants.ELEVATOR_CHAIN_IN_PER_TOOTH) * ClimbConstants.ELEVATOR_CHAIN_SPROCKET_TEETH / ClimbConstants.ELEVATOR_GEARBOX_RATIO;
	}

	private double getElevatorPosition() {
		return rotationsToMeters(elevatorLeaderMotor.getPosition().getValueAsDouble());
	}

	private void raiseTopHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(ClimbConstants.TOP_HOOK_MAX_POSITION));
	}

	private void lowerTopHooks() {
		topClimbHookMotor.setControl(topHookPositionOut.withPosition(ClimbConstants.TOP_HOOK_MIN_POSITION));
	}

	private void raiseBottomHooks() {
		topClimbHookMotor.setControl(bottomHookPositionOut.withPosition(ClimbConstants.BOTTOM_HOOK_MAX_POSITION));
	}

	private void lowerBottomHooks() {
		topClimbHookMotor.setControl(bottomHookPositionOut.withPosition(ClimbConstants.BOTTOM_HOOK_MIN_POSITION));
	}

	private void raiseClimb() {
		moveTo(ClimbConstants.ELEVATOR_MAX_POSITION);
		raiseTopHooks();
		raiseBottomHooks();
	}

	private void lowerClimb() {
		if (getElevatorPosition() < ClimbConstants.ELEVATOR_MAX_POSITION - ClimbConstants.TOLERANCE) {
			System.out.println("WARNING: Elevator not fully deployed before lowering climb");
		}
		moveTo(ClimbConstants.ELEVATOR_MIN_POSITION);
	}

	@Override
	public void periodic() {}
}
