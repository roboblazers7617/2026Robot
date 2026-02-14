// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import yams.math.SmartMath;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**
	 * Constants used to configure the operator controllers.
	 */
	public static class OperatorConstants {
		/**
		 * Controller port index where the driver controller is connected.
		 */
		public static final int DRIVER_CONTROLLER_PORT = 0;
		/**
		 * Controller port index where the operator controller is connected.
		 */
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		/**
		 * Joystick deadband.
		 */
		public static final double DEADBAND = 0.1;
	}

	/**
	 * Constants used to configure logging.
	 * <p>
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will still be logged it just won't be accessible until after the match.
	 * <p>
	 * During testing debug mode should be true to allow for real-time data viewing.
	 */
	public static class LoggingConstants {
		/**
		 * Send logging data to NetworkTables. Data is written to storage when set to false.
		 */
		public static final boolean DEBUG_MODE = true;
		/**
		 * Log all data above specified level.
		 */
		public static final Logged.Importance DEBUG_LEVEL = Logged.Importance.DEBUG;
	}

	/**
	 * Constants used to configure the driver dashboard.
	 */
	public static class DashboardConstants {
		/**
		 * The name of the tab used in Auto.
		 */
		public static final String AUTO_TAB_NAME = "Autonomous";
		/**
		 * The name of the tab used in Teleop.
		 */
		public static final String TELEOP_TAB_NAME = "Teleoperated";
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	}

	/**
	 * Constants used to configure the turret.
	 */
	public static class TurretConstants {
		/**
		 * The CAN ID for the turret motor.
		 */
		public static final int MOTOR_ID = 0;
		/**
		 * The CAN ID for the encoder CANdi.
		 */
		public static final int CANDI_ID = 0;

		/**
		 * The number of teeth on the main turret gear.
		 */
		public static final double TURRET_GEAR_TEETH = 200.0;
		/**
		 * The number of teeth on the gear on the turret motor.
		 */
		public static final double MOTOR_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the encoder pinion.
		 */
		public static final double ENCODER_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the primary encoder's gear.
		 */
		public static final double PRIMARY_ENCODER_GEAR_TEETH = 20.0;
		/**
		 * The number of teeth on the secondary encoder's gear.
		 */
		public static final double SECONDARY_ENCODER_GEAR_TEETH = 21.0;
		/**
		 * The ratio of motor turns to mechanism rotations.
		 */
		public static final double MOTOR_GEAR_RATIO = SmartMath.gearBox(MOTOR_GEAR_TEETH / TURRET_GEAR_TEETH);
		/**
		 * The ratio of encoder pinion turns to mechanism rotations.
		 */
		public static final double ENCODER_GEAR_RATIO = SmartMath.gearBox(ENCODER_GEAR_TEETH / TURRET_GEAR_TEETH);
		/**
		 * The number of primary encoder rotations per motor rotation.
		 */
		public static final double MOTOR_TO_PRIMARY_ENCODER_RATIO = SmartMath.gearBox(MOTOR_GEAR_RATIO, 1 / ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / PRIMARY_ENCODER_GEAR_TEETH);
		/**
		 * The number of encoder rotations per mechanism rotation for the primary encoder.
		 */
		public static final double PRIMARY_ENCODER_RATIO = SmartMath.gearBox(ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / PRIMARY_ENCODER_GEAR_TEETH);
		/**
		 * The number of encoder rotations per mechanism rotation for the secondary encoder.
		 */
		public static final double SECONDARY_ENCODER_RATIO = SmartMath.gearBox(ENCODER_GEAR_RATIO, ENCODER_GEAR_TEETH / SECONDARY_ENCODER_GEAR_TEETH);

		/**
		 * The current limit for the motor.
		 */
		public static final double MOTOR_CURRENT_LIMIT = 40.0;
		/**
		 * The PID kP for the turret closed loop controller.
		 */
		public static final double TURRET_KP = 10.0;
		/**
		 * The PID kI for the turret closed loop controller.
		 */
		public static final double TURRET_KI = 0;
		/**
		 * The PID kD for the turret closed loop controller.
		 */
		public static final double TURRET_KD = 0;
		/**
		 * The PID kS for the turret closed loop controller.
		 */
		public static final double TURRET_KS = 0.25;
		/**
		 * The PID kV for the turret closed loop controller.
		 */
		public static final double TURRET_KV = 0.12;
		/**
		 * The PID kA for the turret closed loop controller.
		 */
		public static final double TURRET_KA = 0.01;
		/**
		 * The peak/cruising velocity of the motion.
		 */
		public static final double CRUISE_VELOCITY = 80;
		/**
		 * The acceleration and deceleration rates during the beginning and end of motion.
		 */
		public static final double ACCELERATION = 160;
		/**
		 * Jerk (derivative of acceleration).
		 */
		public static final double JERK = 1600;

		/**
		 * The lowest angle the turret can rotate to.
		 */
		public static final Angle MINIMUM_ANGLE = Rotations.of(-1.5);
		/**
		 * The highest angle the turret can rotate to.
		 */
		public static final Angle MAXIMUM_ANGLE = Rotations.of(1.5);

		/**
		 * How close does the Turret have to be to its setpoint to be counted as being there.
		 */
		public static final Measure<AngleUnit> SETPOINT_THRESHOLD = Degrees.of(3.0);
	}
}
