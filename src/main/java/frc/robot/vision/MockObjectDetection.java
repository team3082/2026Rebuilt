package frc.robot.vision;

import java.util.ArrayList;
import java.util.Optional;

import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.Odometry;
import frc.robot.utils.Vector2;

/**
 * MockObjectDetection simulates an object detection system for detecting the fuel on the field. 
 * It provides a simple visiblity filiteraying based on a triangular field of view and mutiple strategies for following the target.
 * 
 * <p>This class is intended for simulation and testing purposes only and does
 * not represent real sensor input.</p>
 */
public class MockObjectDetection {

    /**
     * Simulated ball positions in FRC field coordinates.
     */
    private static Vector2[] balls = new Vector2[] {
        new Vector2(292, 248),
        new Vector2(304.5, 248),
        new Vector2(316.75, 248), 
        new Vector2(316.75, 241), 
        new Vector2(322.5, 241), 
        new Vector2(298.25, 229) 
    };

    /**
     * Strategy used to determine which visible ball should be followed.
     */
    public enum VisionStrategy {
        /** Selects the closest visible ball to the robot. */
        NEAREST,

        /** Selects the centroid of all visible balls. */
        CENTER_CLUSTER
    }

    
    /**
     * Returns a target point for the robot to follow based on the selected
     * vision strategy.
     *
     * @param strategy the strategy used to choose a follow point
     * @return an {@link Optional} containing the selected follow point,
     *         or an empty Optional if no balls are visible
     */
    public static Optional<Vector2> getFollowPoint(VisionStrategy strategy) {
        Vector2[] activeBalls = getBallsInLineOfSight();

        if (activeBalls.length == 0) {
            return null; 
        }

        switch (strategy) {
            case NEAREST:
                return Optional.of(getNearestBall(activeBalls));

            case CENTER_CLUSTER:
                return Optional.of(getClusterCenter(activeBalls));

            default:
                return null;
        }
    }

    /**
     * Finds the closest ball to the robot from a list of visible balls.
     *
     * @param balls array of visible ball positions
     * @return the position of the nearest ball
     */
    private static Vector2 getNearestBall(Vector2[] balls) {
        Vector2 robotPos = Odometry.getPosition();

        Vector2 closest = balls[0];
        double minDistSq = robotPos.dist(closest);

        for (Vector2 ball : balls) {
            double distSq = robotPos.dist(ball);
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closest = ball;
            }
        }

        return closest;
    }

    /**
     * Computes the centroid of a group of balls.
     *
     * @param balls array of visible ball positions
     * @return the average (x, y) position of the balls
     */
    private static Vector2 getClusterCenter(Vector2[] balls) {
        double sumX = 0;
        double sumY = 0;

        for (Vector2 ball : balls) {
            sumX += ball.x;
            sumY += ball.y;
        }

        return new Vector2(
            sumX / balls.length,
            sumY / balls.length
        );
    }

    /**
     * Filters simulated balls to those that fall within the robot's
     * triangular field-of-view.
     *
     * @return an array of balls currently visible to the robot
     */
    private static Vector2[] getBallsInLineOfSight() {
        ArrayList<Vector2> visible = new ArrayList<>();

        Vector2[] vertices = getRobotVisionVertices();

        for (Vector2 ball : balls) {
            if (pointInTriangle(ball, vertices[0], vertices[1], vertices[2])) {
                visible.add(ball);
            }
        }

        return visible.toArray(new Vector2[0]);
    }

    /**
     * Computes the vertices of the robot's vision cone in field coordinates.
     * The cone is modeled as an isosceles triangle originating at the robot's
     * position and rotated by the robot's current heading.
     *
     * @return an array of three vertices defining the vision triangle
     */
    private static Vector2[] getRobotVisionVertices() {
        final double halfFovRad = Math.toRadians(30);
        final double viewDistance = 20;

        Vector2[] vertices = {
            new Vector2(0, 0),
            new Vector2(-viewDistance * Math.tan(halfFovRad), viewDistance),
            new Vector2( viewDistance * Math.tan(halfFovRad), viewDistance)
        };

        for (int i = 0; i < vertices.length; i++) {
            vertices[i].rotate(Pigeon.getRotationRad());
            vertices[i].add(Odometry.getPosition());
        }

        return vertices;
    }

    /**
     * Helper method used for determining the relative orientation of three points.
     *
     * @param p1 first point
     * @param p2 second point
     * @param p3 third point
     * @return signed area component used for triangle tests
     */
    private static double sign(Vector2 p1, Vector2 p2, Vector2 p3) {
        return (p1.x - p3.x) * (p2.y - p3.y)
             - (p2.x - p3.x) * (p1.y - p3.y);
    }

    /**
     * Determines whether a point lies inside or on the boundary of a triangle.
     *
     * @param p the point to test
     * @param a first triangle vertex
     * @param b second triangle vertex
     * @param c third triangle vertex
     * @return true if the point is inside or on the triangle, false otherwise
     */
    private static boolean pointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c) {
        double d1 = sign(p, a, b);
        double d2 = sign(p, b, c);
        double d3 = sign(p, c, a);

        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }
}
