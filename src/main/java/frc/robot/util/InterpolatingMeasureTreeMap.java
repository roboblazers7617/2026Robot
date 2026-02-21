package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Interpolating map that interpolates between different Measure keys. This uses linear interpolation.
 */
public class InterpolatingMeasureTreeMap<J extends Measure<U>, U extends Unit, K extends Measure<Q>, Q extends Unit> extends InterpolatingTreeMap<J, K> {
	/**
	 * Creates a new InterpolatingMeasureTreeMap.
	 */
	public InterpolatingMeasureTreeMap() {
		super(new MeasureInverseInterpolable<J, U>(), new MeasureInterpolable<K, Q>());
	}

	/**
	 * An interpolation function for interpolating between Measures.
	 */
	public static class MeasureInterpolable<T extends Measure<U>, U extends Unit> implements Interpolator<T> {
		@Override
		@SuppressWarnings("unchecked")
		public T interpolate(T startValue, T endValue, double t) {
			return (T) startValue.plus(endValue.minus(startValue).times(t));
		}
	}

	/**
	 * An inverse interpolation function for interpolating between Measures.
	 */
	public static class MeasureInverseInterpolable<T extends Measure<U>, U extends Unit> implements InverseInterpolator<T> {
		@Override
		public double inverseInterpolate(T startValue, T endValue, T q) {
			return q.minus(startValue).baseUnitMagnitude() / endValue.minus(startValue).baseUnitMagnitude();
		}
	}
}
