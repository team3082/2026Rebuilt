package frc.robot.utils.trajectories;

import frc.robot.utils.Vector2;

public class ProfiledPoint {
    private Vector2 position;
    private Vector2 velocity;
    private double curvature;
    private double acceleration;
    private double time;
    private double distance;

    public ProfiledPoint() {
        this.position = new Vector2(0, 0);
        this.velocity = new Vector2(0, 0);
        this.curvature = 0;
        this.acceleration = 0;
        this.time = 0;
        this.distance = 0;
    }

    public ProfiledPoint(Vector2 position, Vector2 velocity, double curvature, double acceleration, double time, double distance) {
        this.position = position;
        this.velocity = velocity;
        this.curvature = curvature;
        this.acceleration = acceleration;
        this.time = time;
        this.distance = distance;
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

    public double getTime() {
        return time;
    }

    public double getDistance() {
        return distance;
    }
    
    public void setPosition(Vector2 position) {
        this.position = position;
    }

    public void setVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }
}
