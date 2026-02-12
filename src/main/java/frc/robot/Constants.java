// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;

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

	public static class ClimbConstants {
		public final static int ELEVATOR_LEADER_MOTOR_CAN_ID;
		public final static int ELEVATOR_FOLLOWER_MOTOR_CAN_ID;
		public final static int TOP_HOOK_MOTOR_CAN_ID;
		public final static int BOTTOM_HOOK_MOTOR_CAN_ID;

		public final static int TOP_HOOK_ENCODER_CAN_ID;
		public final static int BOTTOM_HOOK_ENCODER_CAN_ID;

		public final static double KS;
		public final static double KV;
		public final static double KG;
		public final static double KA;

		public final static double KP;
		public final static double KI;
		public final static double KD;

		/**
		 * Elevator kMinOutput as a percentage.
		 */
		public static final double KMIN_OUTPUT = -1.0;
		/**
		 * Elevator kMaxOutput as a percentage.
		 */
		public static final double KMAX_OUTPUT = 1.0;
		/**
		 * Maximum velocity in m/s.
		 */
		public static final double MAX_VELOCITY;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		public static final double MAX_ACCELERATION;

		/**
		 * Maximum velocity in m/s.
		 */
		public static final double MAX_HOOK_VELOCITY;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		public static final double MAX_HOOK_ACCELERATION;

		/**
		 * Maximum position in meters.
		 */
		public static final double MAX_POSITION;
		/**
		 * Minimum position in meters.
		 */
		public static final double MIN_POSITION = 0.0;
		/**
		 * Zero offset, meters.
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;
		/**
		 * Tolerance for the target to be considered reached in meters.
		 */
		public static final double TOLERANCE = .02;

		/*
		 * the IN/link of the chain driving the elevator
		 */
		public static final double ELEVATOR_CHAIN_IN_PER_TOOTH = .25;
		/*
		 * the amount of teeth on the sprocket driving the elevator
		 */
		public static final double ELEVATOR_CHAIN_SPROCKET_TEETH;
		/*
		 * the gearbox ratio of the elevator
		 */
		public static final double ELEVATOR_GEARBOX_RATIO;
	}
}
