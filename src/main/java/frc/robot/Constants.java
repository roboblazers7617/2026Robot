// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

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

	/**
	 * Constants for the drivetrain class
	 */
	public static class DrivetrainConstants {
		/**
		 * Maximum speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = FeetPerSecond.of(14.5).in(MetersPerSecond);
		/**
		 * Directory that contains the YAGSL configuration.
		 */
		public static final File CONFIG_DIR = new File(Filesystem.getDeployDirectory(), "swerve");
		/**
		 * YAGSL telemetry verbosity when in debug mode.
		 */
		// public static final TelemetryVerbosity TELEMETRY_VERBOSITY_DEBUG = TelemetryVerbosity.HIGH;
		// /**
		// * YAGSL telemetry verbosity when in normal mode.
		// */
		// public static final TelemetryVerbosity TELEMETRY_VERBOSITY_NORMAL = TelemetryVerbosity.POSE;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in fast mode.
		 */
		public static final double TRANSLATION_SCALE_FAST = 1;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in normal mode.
		 */
		public static final double TRANSLATION_SCALE_NORMAL = 0.8;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in slow mode.
		 */
		public static final double TRANSLATION_SCALE_SLOW = 0.3;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain when in slide mode.
		 */
		public static final double TRANSLATION_SCALE_SLIDE = 0.3;
		/**
		 * Starting pose.
		 */
		public static final Pose2d STARTING_POSITION = new Pose2d(new Translation2d(Meters.of(1), Meters.of(4)), Rotation2d.fromDegrees(0));
		/**
		 * Enables {@link swervelib.SwerveDrive#headingCorrection heading correction}. Should only be used while controlling the robot via angle.
		 */
		public static final boolean ENABLE_HEADING_CORRECTION = false;
		/**
		 * Enables {@link swervelib.parser.SwerveModuleConfiguration#useCosineCompensator cosine compensation}.
		 */
		public static final boolean ENABLE_COSINE_COMPENSATION = false;

		/**
		 * Angular velocity skew correction configuration.
		 *
		 * @see swervelib.SwerveDrive#setAngularVelocityCompensation
		 */
		public static final class AngularVelocityCompensation {
			/**
			 * Enables angular velocity correction in teleop.
			 */
			public static final boolean USE_IN_TELEOP = true;
			/**
			 * Enables angular velocity correction in autonomous.
			 */
			public static final boolean USE_IN_AUTO = true;
			/**
			 * The angular velocity coefficient.
			 */
			public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.1;
		}
	}
}
