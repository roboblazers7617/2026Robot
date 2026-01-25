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
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
	}

	public static class HopperConstants {
		public final static int LEADER_CAN_ID = 0;
		public final static int FOLLOWER_CAN_ID = 0;
		public final static double KP = 0.15;
		public final static double KI = 0;
		public final static double KD = 0;
		public final static double KV = 0.115;
		public final static double KS = 0;
		public static final double RPS = 20.0;
		public static final double CRUISE_VELOCITY = RPS * 0.8; // 0.8 is 80% effeciancy
		public static final double ACCELERATION = 2.0 * CRUISE_VELOCITY;
		public final static boolean ENABLE_STATOR_LIMIT = true;
		public final static double STATOR_CURRENT_LIMIT = 40;
		public final static boolean ENABLE_SUPPLY_LIMIT = false;
		public final static double SUPPLY_CURRENT_LIMIT = 40;
		public final static double FAST_SPEED = CRUISE_VELOCITY;
		public final static double SLOW_SPEED = CRUISE_VELOCITY / 2.0;
		public final static double COAST_SPEED = CRUISE_VELOCITY / 4.0;
	}

	public static class HopperConstants {
		public final static int BIG_SPINNY_CAN_ID = 1;
		public final static int LITTLE_SPINNY_CAN_ID = 2;
		public final static double KP = 0.15;
		public final static double KI = 0;
		public final static double KD = 0;
		public final static double KV = 0.115;
		public final static double KS = 0;
		public static final double RPS = 20.0;
		public static final double CRUISE_VELOCITY = RPS * 0.8; // 0.8 is 80% effeciancy
		public static final double ACCELERATION = 2.0 * CRUISE_VELOCITY;
		public final static boolean ENABLE_STATOR_LIMIT = true;
		public final static double STATOR_CURRENT_LIMIT = 40;
		public final static boolean ENABLE_SUPPLY_LIMIT = false;
		public final static double SUPPLY_CURRENT_LIMIT = 40;
	}
}
