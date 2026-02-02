package frc.robot.utils.trajectories;

import frc.robot.utils.Vector2;

/**
 * Represents a cubic Bezier curve defined by four control points in 2D space.
 * Provides methods to sample points along the curve.
 */
public class CubicBezierCurve extends Curve {
    private final Vector2 p0; 
    private final Vector2 p1;  
    private final Vector2 p2;  
    private final Vector2 p3; 

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
    }

    /**
     * Gets a point on the cubic Bezier curve at parameter t.
     * @param t Parameter between 0 and 1 representing position along the curve.
     * @return Vector2 point on the curve at parameter t.
     */
    @Override
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
}