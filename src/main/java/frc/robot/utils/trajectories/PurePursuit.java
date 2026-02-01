package frc.robot.utils.trajectories;

import frc.robot.utils.Vector2;

/**
 * Pure Pursuit path follower for a RobotPath, has a currentT progress value along 
 * the path alongside methods to get the target point, heading, and drive vector.
 */
public class PurePursuit {
    private RobotPath path;
    private double currentT;
    
    /**
     * Create a PurePursuit follower for a given path
     * @param path The path to follow
     */
    public PurePursuit(RobotPath path) {
        this.path = path;
        this.currentT = 0.0;
    }
    
    /**
     * Update the follower with current robot position
     * This should be called each loop iteration
     * @param currentPos Current robot position
     */
    public void update(Vector2 currentPos) {
        currentT = findClosestT(currentPos);
    }
    
    /**
     * Get the target point on the path using pure pursuit algorithm
     * @param currentPos Current robot position
     * @param lookAheadDistance How far ahead to look on the path
     * @return Target point to drive towards
     */
    public Vector2 getTargetPoint(Vector2 currentPos, double lookAheadDistance) {
        double targetT = findLookAheadT(currentPos, lookAheadDistance);
        return path.getPointAt(targetT);
    }
    
    /**
     * Get the heading angle to the target point
     * @param currentPos Current robot position
     * @param lookAheadDistance How far ahead to look on the path
     * @return Angle in radians to the target point
     */
    public double getHeading(Vector2 currentPos, double lookAheadDistance) {
        Vector2 targetPoint = getTargetPoint(currentPos, lookAheadDistance);
        Vector2 toTarget = targetPoint.sub(currentPos);
        return Math.atan2(toTarget.y, toTarget.x);
    }
    
    /**
     * Get normalized drive vector pointing towards target
     * @param currentPos Current robot position
     * @param lookAheadDistance How far ahead to look on the path
     * @return Normalized vector pointing to target
     */
    public Vector2 getDriveVector(Vector2 currentPos, double lookAheadDistance) {
        Vector2 targetPoint = getTargetPoint(currentPos, lookAheadDistance);
        return targetPoint.sub(currentPos).norm();
    }
    
    /**
     * Find the closest t value on the path from the current position onward
     */
    private double findClosestT(Vector2 currentPos) {
        int numSamples = path.getPointCount();
        double closestT = currentT;
        double closestDist = Double.MAX_VALUE;
        
        int startIdx = (int)(currentT * (numSamples - 1));
        
        for (int index = startIdx; index < numSamples; index++) {
            double t = (double) index / (numSamples - 1);
            Vector2 point = path.getPointAt(t);
            double dist = point.sub(currentPos).mag();
            
            if (dist < closestDist) {
                closestDist = dist;
                closestT = t;
            } else {
                break;
            }
        }
        
        return closestT;
    }
    
    /**
     * Find t value that is lookAheadDistance away from current position
     */
    private double findLookAheadT(Vector2 currentPos, double lookAheadDistance) {
        int numSamples = path.getPointCount();
        int startIdx = (int)(currentT * (numSamples - 1));
        
        for (int index = startIdx; index < numSamples; index++) {
            double t = (double) index / (numSamples - 1);
            Vector2 point = path.getPointAt(t);
            double dist = point.sub(currentPos).mag();
            
            if (dist >= lookAheadDistance) {
                return t;
            }
        }
        
        return 1.0;
    }
    
    /**
     * Check if robot has reached the end of the path
     * @param currentPos Current robot position
     * @param threshold Distance threshold to consider "reached"
     * @return true if within threshold of end point
     */
    public boolean isAtEnd(Vector2 currentPos, double threshold) {
        Vector2 endPoint = path.getEnd();
        return endPoint.sub(currentPos).mag() < threshold;
    }
    
    /**
     * Get the current progress along the path (0-1)
     * @return Cached t value of closest point (0-1)
     */
    public double getProgress() {
        return currentT;
    }
    
    /**
     * Reset the follower to start of path
     */
    public void reset() {
        currentT = 0.0;
    }
    
    /**
     * Set a new path to follow
     * @param newPath The new path
     */
    public void setPath(RobotPath newPath) {
        this.path = newPath;
        reset();
    }
    
    /**
     * Get the current path being followed
     * @return The current path
     */
    public RobotPath getPath() {
        return path;
    }
}