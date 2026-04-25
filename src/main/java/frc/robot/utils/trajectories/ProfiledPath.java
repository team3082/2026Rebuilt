package frc.robot.utils.trajectories;

import java.util.ArrayList;

import frc.robot.utils.Vector2;

public class ProfiledPath {
    private ArrayList<ProfiledPoint> profiledPoints;

    public ProfiledPath(ArrayList<ProfiledPoint> profiledPoints) {
        this.profiledPoints = profiledPoints;
    }
    
    public ProfiledPoint getPointAtTime(double t){
        int closestIndex = 0;
        for (int index = 0; index < profiledPoints.size() - 1; index++) {
            if (profiledPoints.get(index).getTime() <= t) {
                closestIndex = index;
            } else {
                break;
            }
        }
        ProfiledPoint p0 = profiledPoints.get(closestIndex);
        ProfiledPoint p1 = profiledPoints.get(closestIndex + 1);
        
     
        // Calculate how far we are into the segment (0.0 to 1.0).
        double ratio = (t - p0.getTime()) / (p1.getTime() - p0.getTime());

        // Linearly interpolate all values.
        Vector2 position = p0.getPosition().mul(1 - ratio).add(p1.getPosition().mul(ratio));
        Vector2 velocity = p0.getVelocity().mul(1 - ratio).add(p1.getVelocity().mul(ratio));
        double curvature = p0.getCurvature() * (1 - ratio) + p1.getCurvature() * ratio;
        double acceleration = p0.getAcceleration() * (1 - ratio) + p1.getAcceleration() * ratio;
        double distance = p0.getDistance() * (1 - ratio) + p1.getDistance() * ratio;
        double heading = interpolateHeading(p0.getHeading(), p1.getHeading(), ratio);
        double rotVel = p0.getRotationalVelocity() * (1 - ratio) + p1.getRotationalVelocity() * ratio;

        ProfiledPoint result = new ProfiledPoint(position, velocity, curvature, acceleration, t, distance);
        result.setHeading(heading);
        result.setRotationalVelocity(rotVel);
        return result;
    }
    
    /**
     * Returns the shortest distance from the given robot position to this path.
     * The path is treated as a polyline connecting the sequence of ProfiledPoints.
     * If the path has fewer than 2 points, distance to the single point (or 0 if empty)
     * is returned.
     *
     * @param robotPos robot position in the same coordinate frame as ProfiledPoint positions
     * @return shortest distance (double)
     */
    public double getClosestPointToRobotDistance(Vector2 robotPos) {
        if (profiledPoints == null || profiledPoints.isEmpty()) return 0.0;
        if (profiledPoints.size() == 1) return profiledPoints.get(0).getPosition().dist(robotPos);

        double minDist = Double.POSITIVE_INFINITY;

        for (int i = 0; i < profiledPoints.size() - 1; i++) {
          
        }

        return minDist;
    }

    /** Convenience overload accepting coordinates */
    public double getClosestPointToRobotDistance(double rx, double ry) {
        return getClosestPointToRobotDistance(new Vector2(rx, ry));
    }

    /**
     * Finds and returns a ProfiledPoint on this path that is closest to the given robot
     * position. The returned ProfiledPoint is an interpolated point on the segment
     * between the two nearest sample points (so time, distance and other values
     * are interpolated linearly).
     *
     * @param robotPos robot position in the same coordinate frame as ProfiledPoint positions
     * @return interpolated ProfiledPoint nearest to robotPos, or null if path is empty
     */
    public ProfiledPoint getClosestProfiledPoint(Vector2 robotPos) {
        if (profiledPoints == null || profiledPoints.isEmpty()) return null;
        if (profiledPoints.size() == 1) {
            ProfiledPoint single = profiledPoints.get(0);
            // return a copy
            ProfiledPoint copy = new ProfiledPoint(single.getPosition().copy(), single.getVelocity() != null ? single.getVelocity().copy() : new Vector2(), single.getCurvature(), single.getAcceleration(), single.getTime(), single.getDistance());
            copy.setHeading(single.getHeading());
            copy.setRotationalVelocity(single.getRotationalVelocity());
            return copy;
        }

        double minDist = Double.POSITIVE_INFINITY;
        int bestIndex = 0;
        double bestT = 0.0;

        for (int i = 0; i < profiledPoints.size() - 1; i++) {
            Vector2 a = profiledPoints.get(i).getPosition();
            Vector2 b = profiledPoints.get(i + 1).getPosition();
            Vector2 ab = b.sub(a);
            Vector2 ar = robotPos.sub(a);

            double abLenSq = ab.x * ab.x + ab.y * ab.y;
            double t = 0.0;
            if (abLenSq > 0) {
                t = (ar.x * ab.x + ar.y * ab.y) / abLenSq;
            }
            t = Math.max(0.0, Math.min(1.0, t));

            Vector2 closest = a.add(ab.mul(t));
            double dist = closest.dist(robotPos);
            if (dist < minDist) {
                minDist = dist;
                bestIndex = i;
                bestT = t;
            }
        }

        ProfiledPoint p0 = profiledPoints.get(bestIndex);
        ProfiledPoint p1 = profiledPoints.get(bestIndex + 1);

        // Interpolate fields
        Vector2 position = p0.getPosition().mul(1 - bestT).add(p1.getPosition().mul(bestT));
        Vector2 velocity = (p0.getVelocity() != null && p1.getVelocity() != null)
                ? p0.getVelocity().mul(1 - bestT).add(p1.getVelocity().mul(bestT))
                : new Vector2(0, 0);
        double curvature = p0.getCurvature() * (1 - bestT) + p1.getCurvature() * bestT;
        double acceleration = p0.getAcceleration() * (1 - bestT) + p1.getAcceleration() * bestT;
        double time = p0.getTime() * (1 - bestT) + p1.getTime() * bestT;
        double distance = p0.getDistance() * (1 - bestT) + p1.getDistance() * bestT;
        double heading = interpolateHeading(p0.getHeading(), p1.getHeading(), bestT);
        double rotVel = p0.getRotationalVelocity() * (1 - bestT) + p1.getRotationalVelocity() * bestT;

        ProfiledPoint result = new ProfiledPoint(position, velocity, curvature, acceleration, time, distance);
        result.setHeading(heading);
        result.setRotationalVelocity(rotVel);
        return result;
    }

    /** Convenience overload accepting coordinates */
    public ProfiledPoint getClosestProfiledPoint(double rx, double ry) {
        return getClosestProfiledPoint(new Vector2(rx, ry));
    }
    
    public Vector2 getStartPoint() {
       return profiledPoints.get(0).getPosition();
    }

    public double getStartHeading() {
        return profiledPoints.get(0).getHeading();
    }

    public double getDuration() {
        return profiledPoints.get(profiledPoints.size()-1).getTime();
    }

    private static double interpolateHeading(double startHeading, double endHeading, double ratio) {
        double delta = endHeading - startHeading;
        while (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return startHeading + delta * ratio;
    }
}
