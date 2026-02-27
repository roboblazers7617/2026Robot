package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A utility class used for sending alerts to Elastic and Driver Station.
 */
public class AlertUtil {
	/**
	 * The notification level for an Alert.
	 */
	public enum AlertLevel {
		/**
		 * Informational message.
		 */
		INFO,
		/**
		 * Warning message.
		 */
		WARNING,
		/**
		 * Error message.
		 * <p>
		 * When using this notification level, a stack trace will also be output.
		 */
		ERROR,
	}

	/**
	 * Sends a notification to Elastic and Driver Station.
	 *
	 * @param level
	 *            The notification level for the notification.
	 * @param title
	 *            The title for the notification. This only gets displayed on Elastic, not on Driver Station.
	 * @param message
	 *            The description for the notification.
	 */
	public static void sendNotification(AlertLevel level, String title, String message) {
		switch (level) {
			case INFO:
				System.out.println(message);
				break;

			case WARNING:
				DriverStation.reportWarning(message, false);
				break;

			case ERROR:
				DriverStation.reportError(message, true);
				break;
		}

		Elastic.Notification notification = new Elastic.Notification()
				.withTitle(title)
				.withDescription(message)
				.withDisplayMilliseconds(5000);

		switch (level) {
			case INFO:
				notification.withLevel(Elastic.Notification.NotificationLevel.INFO);
				break;

			case WARNING:
				notification.withLevel(Elastic.Notification.NotificationLevel.WARNING);
				break;

			case ERROR:
				notification.withLevel(Elastic.Notification.NotificationLevel.ERROR);
				break;
		}

		Elastic.sendNotification(notification);
	}
}
