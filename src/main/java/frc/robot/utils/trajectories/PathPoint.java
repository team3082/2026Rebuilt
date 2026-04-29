package frc.robot.utils.trajectories;

import frc.robot.utils.Vector2;

public class PathPoint {
    public Vector2 position = new Vector2(0, 0);
    public Vector2 velocity = new Vector2(0, 0);
    public double curvature = 0;
    public double acceleration = 0;
    public double s = 0;
    public double time = 0;
    public double heading = 0;
    public double rotationalVelocity = 0;

    public PathPoint() {
    }

    public Vector2 getPosition() {
        return position;
    }

    public Vector2 getVelocity() {
        return velocity;
    }

    public double getCurvature() {
        return curvature;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getDistance() {
        return s;
    }

    public double getTime() {
        return time;
    }

    public double getHeading() {
        return heading;
    }

    public double getRotationalVelocity() {
        return rotationalVelocity;
    }

    public PathPoint copy() {
        PathPoint p = new PathPoint();
        p.position = new Vector2(position.x, position.y);
        p.velocity = new Vector2(velocity.x, velocity.y);
        p.curvature = curvature;
        p.acceleration = acceleration;
        p.s = s;
        p.time = time;
        p.heading = heading;
        p.rotationalVelocity = rotationalVelocity;
        return p;
    }
}
