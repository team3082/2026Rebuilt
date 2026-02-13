package frc.robot.utils.trajectories;

import frc.robot.Constants;
import frc.robot.utils.Vector2;

/**
 * Represents a cubic Bezier curve defined by four control points in 2D space.
 * Provides methods to sample points along the curve.
 */
public class CubicBezierCurve {
    private final Vector2 p0; 
    private final Vector2 p1;  
    private final Vector2 p2;  
    private final Vector2 p3; 

    private final Vector2[] points;
    private final double[] curvatures;
    private final double[] tValues;

    /**
     * Creates a cubic Bezier curve defined by four control points.
     * @param p0 Vector2 of the  first control point.
     * @param p1 Vector2 of the second control point.
     * @param p2 Vector2 of the third control point.
     * @param p3 Vector2 of the fourth control point.
     */
    public CubicBezierCurve(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) {
        if (p0 == null || p1 == null || p2 == null || p3 == null) {
            throw new IllegalArgumentException("Control points cannot be null");
        }

        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        this.points = new Vector2[Constants.TrajectoryConstants.CURVE_RESOLUTION]; 
        this.curvatures = new double[Constants.TrajectoryConstants.CURVE_RESOLUTION]; 
        this.tValues = new double[Constants.TrajectoryConstants.CURVE_RESOLUTION];

        for (int i = 0; i < Constants.TrajectoryConstants.CURVE_RESOLUTION; i++) {
            double t = (double)i / (Constants.TrajectoryConstants.CURVE_RESOLUTION - 1);
            points[i] = getPoint(t);
            curvatures[i] = getCurvatureAt(t);
            tValues[i] = t;
        }

    }

    /**
     * Gets the precomputed points along the curve.
     * @return Array of points along the curve.
     */
    public Vector2[] getPoints() {
        return points;
    }

    /**
     * Gets the precomputed curvatures along the curve.
     * @return Array of curvature values at precomputed points along the curve.
     */
    public double[] getCurvatures() {
        return curvatures;
    }

    /**
     * Gets a point on the cubic Bezier curve at parameter t.
     * @param t Parameter between 0 and 1 representing position along the curve.
     * @return Vector2 point on the curve at parameter t.
     */
    public Vector2 getPoint(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException(
                "Parameter t must be in range [0, 1], got: " + t
            );
        }
        
        double x = (Math.pow(1 - t,3) * p0.x) +
        (3 * Math.pow(1 - t,2) * t * p1.x) +
        (3 * (1 - t) * Math.pow(t,2) * p2.x) +
        (Math.pow(t,3) * p3.x);

        double y = (Math.pow(1 - t,3) * p0.y) +
        (3 * Math.pow(1 - t,2) * t * p1.y) +
        (3 * (1 - t) * Math.pow(t,2) * p2.y) +
        (Math.pow(t,3) * p3.y);

        Vector2 r = new Vector2(x, y);
        return r;
    }

    /**
     * Calculates the curvature of the cubic Bezier curve at parameter t.
     * @param t Parameter between 0 and 1 representing position along the curve.
     * @return Curvature value at parameter t.
     */
    public double getCurvatureAt(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException(
                "Parameter t must be in range [0, 1], got: " + t
            );
        }

        Vector2 d1 = p1.sub(p0).mul(3 * Math.pow(1 - t, 2))
                     .add(p2.sub(p1).mul(6 * (1 - t) * t))
                     .add(p3.sub(p2).mul(3 * Math.pow(t, 2)));

        Vector2 d2 = p2.sub(p1).mul(6 * (1 - t))
                     .add(p3.sub(p2).mul(6 * t));

        double numerator = d1.x * d2.y - d1.y * d2.x;
        double denominator = Math.pow(d1.x * d1.x + d1.y * d1.y, 1.5);

        if (denominator == 0) {
            return 0;
        }

        return numerator / denominator;
    }
    
    /**
     * Gets the first control point P0.
     * @return Vector2 representing P0.
     */
    public Vector2 getP0() { return p0; }

    /**
     * Gets the first control point P1.
     * @return Vector2 representing P1.
     */
    public Vector2 getP1() { return p1; }

    /**
     * Gets the first control point P2.
     * @return Vector2 representing P2.
     */
    public Vector2 getP2() { return p2; }

    /**
     * Gets the first control point P3.
     * @return Vector2 representing P3.
     */
    public Vector2 getP3() { return p3; }

    /**
     * 
     * @return
     */
    public double[] getTValues() {
        return tValues;
    }
}