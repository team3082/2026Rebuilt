package frc.robot.utils.trajectories;

import frc.robot.Constants;
import frc.robot.utils.Vector2;

/**
 * Represents a parametric curve in 2D space. 
 * Provides methods to sample points along the curve and calculate arc length.
 * All curves must implement this class.
 */
public abstract class Curve {
    private Vector2[] cachedPoints = null;
    private double cachedArcLength = -1;
    
    /**
     * Returns a point on the curve corresponding to the parameter t ∈ [0, 1].
     * 
     * @param t Fraction along the curve (0 = start, 1 = end)
     * @return Vector2 representing the point at parameter t
     */
    public abstract Vector2 getPoint(double t);

    /**
     * Returns an array of points sampled along the curve.
     * Uses a fixed resolution defined by {@link Constants.TrajectoryConstants#CURVE_RESOLUTION}.
     * <p>
     * Caches the points after the first computation for efficiency.
     * 
     * @return Array of Vector2 points along the curve
     */
    public Vector2[] getPointsOnCurve() {
        if (cachedPoints == null) {
            cachedPoints = computePointsOnCurve();
        }
        return cachedPoints;
    }
    
    /**
     * Computes the sampled points along the curve.
     * This method is used internally by {@link #getPointsOnCurve()}.
     * 
     * @return Array of Vector2 points along the curve
     */
    private Vector2[] computePointsOnCurve() {
        int resolution = Constants.TrajectoryConstants.CURVE_RESOLUTION;
        Vector2[] curvePoints = new Vector2[resolution];

        for (int index = 0; index < resolution; index++) {
            double t = (double) index / (resolution - 1);
            curvePoints[index] = getPoint(t);
        }

        return curvePoints;
    }

    /**
     * Returns the total arc length of the curve, computed as the sum of
     * linear distances between sampled points.
     * <p>
     * Caches the result after the first computation.
     * 
     * @return Arc length of the curve
     */
    public double getArcLength() {
        if (cachedArcLength < 0) {
            cachedArcLength = computeArcLength();
        }
        return cachedArcLength;
    }

    /**
     * Computes the arc length of the curve by summing distances
     * between sampled points. This method is used internally by {@link #getArcLength()}.
     * 
     * @return Arc length of the curve
     */
    private double computeArcLength() {
        int samples = Constants.TrajectoryConstants.CURVE_RESOLUTION;
        double length = 0;
        Vector2 prevPoint = getPoint(0);
        
        for (int index = 1; index < samples; index++) { 
            double t = (double) index / (samples - 1);
            Vector2 currentPoint = getPoint(t);
            length += prevPoint.dist(currentPoint);
            prevPoint = currentPoint;
        }
        
        return length;
    }

    /**
     * Invalidates the cached sampled points and arc length.
     * Call this method if the curve's definition changes.
     */
    public void invalidateCache() {
        cachedPoints = null;
        cachedArcLength = -1;
    }
}
