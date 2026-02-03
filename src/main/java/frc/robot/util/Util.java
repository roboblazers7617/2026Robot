package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * General utilities.
 */
public class Util {
	/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return
	 *         true if the red alliance, false if blue. Defaults to false if none is available.
	 */
	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}

	/**
	 * Records a metadata string to the logs and to NetworkTables to be viewed in AdvantageScope.
	 *
	 * @param key
	 *            The key to record the metadata under.
	 * @param value
	 *            The value to record.
	 */
	public static void recordMetadata(String key, String value) {
		String path = String.format("/Metadata/%s", key);

		StringPublisher publisher = NetworkTableInstance.getDefault()
				.getStringTopic(path)
				.publish();
		publisher.set(value);

		SignalLogger.writeString(key, value);
	}
}
