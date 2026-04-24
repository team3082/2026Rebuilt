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
