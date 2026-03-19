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
		public final static int CIB_MOTOR_CAN_ID = 50;

		public final static double CIB_KS = .265;
		public final static double CIB_KV = 6;
		public final static double CIB_KG = 0;
		public final static double CIB_KA = 0;

		public final static double CIB_KP = 1.7;
		public final static double CIB_KI = 0;
		public final static double CIB_KD = 0;

		public final static double UP_SPEED = .25;
		public final static double DOWN_SPEED = -UP_SPEED;

		/**
		 * the speed of the motor when zeroing the encoder, from 0 to -1
		 */
		public final static double ENCODER_ZERO_SPEED = -.25;

		/**
		 * kMinOutput as a percentage.
		 */
		public static final double KMIN_OUTPUT = -1.0;
		/**
		 * kMaxOutput as a percentage.
		 */
		public static final double KMAX_OUTPUT = 1.0;

		/**
		 * Max position in meters.
		 */
		public static final double CIB_MAX_POSITION = 0.2071581582; // 8.15in to m

		/*
		 * Max rotations of the climb motor
		 */
		public static final double CIB_MAX_ROTATIONS = 108;
		/**
		 * Zero offset, meters.
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;

		/**
		 * Tolerance for the target to be considered reached in rotations.
		 */
		public static final double TOLERANCE = .2;
		/*
		 * the gearbox ratio of the climber, if 2:1 should be .5, as every rotation of the motor equates to .5 rotations of the shaft
		 */
		public static final double CIB_GEARBOX_RATIO = 1.0 / 36.0;

		/*
		 * current limits
		 */
		public static final double CIB_SUPPLY_CURRENT_LIMIT = 60;
		public static final double CIB_STATOR_CURRENT_LIMIT = 90;

		public static final double CIB_SUPPLY_CURRENT_LOWER_LIMIT = 40;
		public static final double CIB_SUPPLY_CURRENT_LIMIT_LOWER_TIME = .1;

		/**
		 * Maximum velocity in m/s.
		 */
		public static final double MAX_VELOCITY = (100.0 / CIB_GEARBOX_RATIO) * .8;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		public static final double MAX_ACCELERATION = 4 * MAX_VELOCITY;
	}
}
