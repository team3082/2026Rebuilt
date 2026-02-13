package frc.robot.utils.trajectories;

import java.util.ArrayList;

import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class ProfiledPath {
    private ArrayList<ProfiledPoint> profiledPoints;

    public ProfiledPath(ArrayList<ProfiledPoint> profiledPoints) {
        this.profiledPoints = profiledPoints;
    }
    
    public ProfiledPoint getPointAtTime(double t){
        double minDist = Double.MAX_VALUE;
        int closestIndex = -1;

        for (int index = 0; index < profiledPoints.size(); index++) {
            ProfiledPoint p = profiledPoints.get(index);
            double dist = Math.abs(p.getTime() - t);
            if (dist < minDist) {
                minDist = dist;
                closestIndex = index;
            }
        }

        if (closestIndex == 0) {
            return profiledPoints.get(0);
        }

        ProfiledPoint p0 = profiledPoints.get(closestIndex - 1); 
        ProfiledPoint p1 = profiledPoints.get(closestIndex);
        
     
        // Calculate how far we are into the segment (0.0 to 1.0).
        double ratio = (t - p0.getTime()) / (p1.getTime() - p0.getTime());

        // Linearly interpolate all values.
        Vector2 position = p0.getPosition().mul(1 - ratio).add(p1.getPosition().mul(ratio));
        Vector2 velocity = p0.getVelocity().mul(1 - ratio).add(p1.getVelocity().mul(ratio));
        double curvature = p0.getCurvature() * (1 - ratio) + p1.getCurvature() * ratio;
        double acceleration = p0.getAcceleration() * (1 - ratio) + p1.getAcceleration() * ratio;
        double distance = p0.getDistance() * (1 - ratio) + p1.getDistance() * ratio;  
        double t_value = p0.getTValue()  * (1 - ratio) + p1.getTValue() * ratio;

        return new ProfiledPoint(position, velocity, curvature, acceleration, t, distance, t_value);
    }

    public static ProfiledPath generateProfiledPath(RobotPath path, double maxVelocity, double maxAcceleration, double maxLateralAcceleration) {
        if (path.getPoints().size() < 2) {
            return new ProfiledPath(new ArrayList<>());
        }
        
        ArrayList<ProfiledPoint> profiledPoints = initializeProfiledPoints(path);

        // This will hold the scalar velocity at each point.
        ArrayList<Double> scalarVelocities = new ArrayList<>();
        for (int i = 0; i < profiledPoints.size(); i++) {
            scalarVelocities.add(0.0);
        }

        // 1. Lateral Constraint Pass
        for (int i = 0; i < profiledPoints.size(); i++) {
            double curvature = profiledPoints.get(i).getCurvature();
            double v_lat;
            if (Math.abs(curvature) > 1e-9) {
                v_lat = Math.sqrt(maxLateralAcceleration / Math.abs(curvature));
            } else {
                v_lat = maxVelocity;
            }
            scalarVelocities.set(i, Math.min(v_lat, maxVelocity));
        }

        // 2. Forward Pass (Acceleration)
        scalarVelocities.set(0, 0.0); // Start at rest
        for (int i = 1; i < profiledPoints.size(); i++) {
            double prev_v = scalarVelocities.get(i - 1);
            double ds = profiledPoints.get(i).getDistance() - profiledPoints.get(i - 1).getDistance();
            double v_accel = Math.sqrt(prev_v * prev_v + 2 * maxAcceleration * ds);
            scalarVelocities.set(i, Math.min(scalarVelocities.get(i), v_accel));
        }

        // 3. Backward Pass (Deceleration)
        scalarVelocities.set(profiledPoints.size() - 1, 0.0); 
        for (int i = profiledPoints.size() - 2; i >= 0; i--) {
            double next_v = scalarVelocities.get(i + 1);
            double ds = profiledPoints.get(i + 1).getDistance() - profiledPoints.get(i).getDistance();
            double v_decel = Math.sqrt(next_v * next_v + 2 * maxAcceleration * ds);
            scalarVelocities.set(i, Math.min(scalarVelocities.get(i), v_decel));
        }

        // Calculate  time for each point and the velocity vectors.
        double accumulatedTime = 0.0;
        profiledPoints.get(0).setTime(0.0);
        profiledPoints.get(0).setVelocity(new Vector2(0, 0));

        for (int i = 1; i < profiledPoints.size(); i++) {
            double v_avg = (scalarVelocities.get(i) + scalarVelocities.get(i - 1)) / 2.0;
            double ds = profiledPoints.get(i).getDistance() - profiledPoints.get(i - 1).getDistance();
            double dt = 0.0;
            if (Math.abs(v_avg) > 1e-9) {
                dt = ds / v_avg;
            }
            accumulatedTime += dt;
            profiledPoints.get(i).setTime(accumulatedTime);

            Vector2 direction = profiledPoints.get(i).getPosition().sub(profiledPoints.get(i-1).getPosition()).norm();
            if (direction.mag() == 0 && i > 1) {
                direction = profiledPoints.get(i-1).getPosition().sub(profiledPoints.get(i-2).getPosition()).norm();
            }
            
            profiledPoints.get(i).setVelocity(direction.mul(scalarVelocities.get(i)));
        }

        // Calculate acceleration
        for (int i = 1; i < profiledPoints.size(); i++) {
            double dv = scalarVelocities.get(i) - scalarVelocities.get(i-1);
            double dt = profiledPoints.get(i).getTime() - profiledPoints.get(i-1).getTime();
            double acceleration = 0;
            if (Math.abs(dt) > 1e-9) {
                acceleration = dv / dt;
            }
            profiledPoints.get(i).setAcceleration(acceleration);
        }
        profiledPoints.get(0).setAcceleration(profiledPoints.get(1).getAcceleration());

        return new ProfiledPath(profiledPoints);
    }

    /**
     * Initializes the list of ProfiledPoint objects from the RobotPath.
     * @param path The path to profile.
     * @return An ArrayList of initialized ProfiledPoint objects.
     */
    private static ArrayList<ProfiledPoint> initializeProfiledPoints(RobotPath path) {
        ArrayList<ProfiledPoint> profiledPoints = new ArrayList<>();
        ArrayList<Vector2> points = new ArrayList<>(path.getPoints());
        ArrayList<Double> curvatures = new ArrayList<>(path.getCurvatures());
        ArrayList<Double> tValues = new ArrayList<>(path.getTValues());

        if (points.isEmpty()) {
            return profiledPoints;
        }

        ProfiledPoint startPoint = new ProfiledPoint();
        startPoint.setPosition(points.get(0));
        startPoint.setCurvature(curvatures.get(0));
        startPoint.setDistance(0.0);
        profiledPoints.add(startPoint);
        
        // Initialize ProfiledPoints
        for (int i = 1; i < points.size(); i++) {
            ProfiledPoint p = new ProfiledPoint();
            p.setPosition(points.get(i));
            p.setCurvature(curvatures.get(i));
            p.setTValue(tValues.get(i));
            
            double segmentDistance = points.get(i-1).dist(points.get(i));
            double previousDistance = profiledPoints.get(i-1).getDistance();
            p.setDistance(previousDistance + segmentDistance);


            profiledPoints.add(p);
        }

        return profiledPoints;
    }

    
    public Vector2 getStartPoint() {
       return profiledPoints.get(0).getPosition();
    }

    public double getDuration() {
        return profiledPoints.get(profiledPoints.size()-1).getTime();
    }
}
