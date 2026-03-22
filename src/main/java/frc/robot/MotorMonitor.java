package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.traits.HasTalonSignals;

import edu.wpi.first.units.measure.Temperature;
import frc.robot.Constants.DashboardConstants;
import frc.robot.util.AlertUtil;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Seconds;

/**
 * Util class that handles periodically iterating over all of the motors on the robot and notifying on the dashboard if one goes over a temperature threshold.
 */
public class MotorMonitor {
	private static List<Monitored> devices = new ArrayList<>();

	/**
	 * Adds a motor to the motor monitor.
	 *
	 * @param motor
	 *            The motor to add.
	 */
	public static void addMotor(HasTalonSignals motor) {
		Monitored device = new Monitored();
		device.motor = motor;

		devices.add(device);
	}

	/**
	 * Iterates over all the motors and alerts if they're over their temperature threshold.
	 */
	public static void update() {
		for (Monitored device : devices) {
			HasTalonSignals motor = device.motor;

			Temperature deviceTemp = motor.getDeviceTemp().getValue();
			if (!device.hasNotified && deviceTemp.compareTo(DashboardConstants.MOTOR_WARNING_TEMPERATURE) > 0) {
				AlertUtil.sendNotification(AlertUtil.AlertLevel.WARNING, "Motor Overtemperature", String.format("Motor ID %d over temperature threshold (%.1f \u00B0C)!", motor.getDeviceID(), deviceTemp.in(Celsius)), Seconds.zero());
				device.hasNotified = true;
			} else if (device.hasNotified && deviceTemp.compareTo(DashboardConstants.MOTOR_WARNING_RESET_TEMPERATURE) < 0) {
				device.hasNotified = false;
			}
		}
	}

	/**
	 * A monitored motor.
	 */
	private static class Monitored {
		/**
		 * The object to monitor.
		 */
		public HasTalonSignals motor;
		/**
		 * Has a notification been sent for this device?
		 */
		public boolean hasNotified = false;
	}
}
