// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.BiAlliancePose3d;
import frc.robot.util.PoseUtil;
import frc.robot.util.RectangleUtil;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class AutoConstants {
		public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
		public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
	}

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
		/**
		 * A pose at the center of the field.
		 */
		public static final Pose2d FIELD_CENTER = new Pose2d(FIELD_LAYOUT.getFieldLength() / 2, FIELD_LAYOUT.getFieldWidth() / 2, Rotation2d.kZero);
		/**
		 * A rectangle encompassing the neutral (center) zone of the field.
		 */
		public static final Rectangle2d NEUTRAL_RECTANGLE = new Rectangle2d(FIELD_CENTER, Inches.of(287.0), Meters.of(FIELD_LAYOUT.getFieldWidth()));

		/**
		 * Zones and poses to shoot from and to.
		 */
		public static class ShootingZones {
			/**
			 * A rectangle encompassing the shooting zone for the top half of the neutral rectangle (when looking at the field diagram).
			 */
			public static final Rectangle2d NEUTRAL_ZONE_TOP = new Rectangle2d(FIELD_CENTER.transformBy(new Transform2d(0, -FIELD_LAYOUT.getFieldWidth() / 3.0, Rotation2d.kZero)), NEUTRAL_RECTANGLE.getXWidth(), NEUTRAL_RECTANGLE.getYWidth() / 3.0);
			/**
			 * A rectangle encompassing the shooting zone for the bottom half of the neutral rectangle (when looking at the field diagram).
			 */
			public static final Rectangle2d NEUTRAL_ZONE_BOTTOM = RectangleUtil.flipRectangleY(NEUTRAL_ZONE_TOP);
			/**
			 * The pose to shoot at for the {@link #NEUTRAL_ZONE_TOP top of the neutral rectangle} (when looking at the field diagram).
			 */
			public static final BiAlliancePose3d SHUTTLE_TOP_POSE = BiAlliancePose3d.fromBluePose(new Pose3d(Meters.of(3.0), Meters.of(2.0), Meters.zero(), Rotation3d.kZero), BiAlliancePose3d.InvertY.KEEP_Y);
			/**
			 * The pose to shoot at for the {@link #NEUTRAL_ZONE_BOTTOM bottom of the neutral rectangle} (when looking at the field diagram).
			 */
			public static final BiAlliancePose3d SHUTTLE_BOTTOM_POSE = BiAlliancePose3d.fromBluePose(PoseUtil.flipPoseY(SHUTTLE_TOP_POSE.getBluePose()), BiAlliancePose3d.InvertY.KEEP_Y);

			/**
			 * A rectangle encompassing the shooting zone for the hub on the red alliance.
			 */
			public static final Rectangle2d HUB_ZONE_RED = new Rectangle2d(FIELD_CENTER.transformBy(new Transform2d(Inches.of(234.555).plus(Meters.of(0.1)), Meters.zero(), Rotation2d.kZero)), Inches.of(182.11), Meters.of(5.0));
			/**
			 * A rectangle encompassing the shooting zone for the hub on the blue alliance.
			 */
			public static final Rectangle2d HUB_ZONE_BLUE = RectangleUtil.flipRectangleX(HUB_ZONE_RED);
			/**
			 * The pose to shoot at for the {@link #HUB_ZONE_RED} and {@link #HUB_ZONE_BLUE}.
			 */
			public static final BiAlliancePose3d HUB_POSE = BiAlliancePose3d.fromRedPose(new Pose3d(FIELD_CENTER).transformBy(new Transform3d(Inches.of(143.50), Meters.zero(), Inches.of(72.0), Rotation3d.kZero)), BiAlliancePose3d.InvertY.KEEP_Y);
		}
	}

	public static class DrivetrainConstants {
		/**
		 * Speed Multipliers
		 */

		public static final double MAX_SPEED_MULTIPLIER = 0.5;
		public static final double NORMAL_SPEED_MULTIPLIER = 0.35;
		public static final double SLOW_SPEED_MULTIPLIER = 0.25;
		// for use in speed multiplier ONLY
		public static final double MAX_SPEED_SWERVE = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		// for use in other things that do not need to have maximum speed/speed multiplier
		public static final double MAX_SPEED_DEADBAND = 0.35 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		// will set spinny mode turn speed
		public static final double MAX_ANGULAR_RATE_DEADBAND = 0.5 * RotationsPerSecond.of(0.75).in(RadiansPerSecond);
	}
}
