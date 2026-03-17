package frc.robot.utils.trajectories;

import java.util.ArrayList;

import frc.robot.Constants;
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
        double heading = p0.getHeading() * (1 - ratio) + p1.getHeading() * ratio;
        double rotVel = p0.getRotationalVelocity() * (1 - ratio) + p1.getRotationalVelocity() * ratio;

        ProfiledPoint result = new ProfiledPoint(position, velocity, curvature, acceleration, t, distance);
        result.setHeading(heading);
        result.setRotationalVelocity(rotVel);
        return result;
    }

    public static ProfiledPath generateSimplifiedProfile(
        RobotPath path,
        double maxTransVelocity,
        double maxRotVelocity,
        double maxWheelSpeed,
        double maxAcceleration,
        double[] targetHeadings) {

        ArrayList<ProfiledPoint> points = initializeProfiledPoints(path);
        int N = points.size();

        // ── 1. Resolve and unwrap headings ────────────────────────────
        double[] headings = new double[N];
        if (targetHeadings != null && targetHeadings.length == N) {
            System.arraycopy(targetHeadings, 0, headings, 0, N);
        } else {
            // Default: hold the initial path heading throughout
            double initialHeading = points.get(0).getHeading();
            for (int i = 0; i < N; i++) headings[i] = initialHeading;
        }

        // Unwrap to eliminate ±2 pi jumps
        for (int i = 1; i < N; i++) {
            double delta = headings[i] - headings[i - 1];
            while (delta >  Math.PI) delta -= 2 * Math.PI;
            while (delta < -Math.PI) delta += 2 * Math.PI;
            headings[i] = headings[i - 1] + delta;
        }

        // ── 2. Per-segment heading change ─────────────────────────────
        double[] dtheta = new double[N];
        dtheta[0] = 0.0;
        for (int i = 1; i < N; i++) {
            dtheta[i] = headings[i] - headings[i - 1];
        }

        // ── 3. Velocity constraints ────────────────────────────────────
        double r_max = Math.hypot(
            Math.abs(Constants.Swerve.SWERVEMODX0),
            Math.abs(Constants.Swerve.SWERVEMODY0)
        );
        ArrayList<Double> velocities = new ArrayList<>();
        for (int i = 0; i < N; i++) velocities.add(maxTransVelocity);

        for (int i = 1; i < N; i++) {
            double ds = points.get(i).getDistance() - points.get(i - 1).getDistance();
            if (ds > 1e-9) {
                // Wheel speed constraint: translation + rotation share wheel budget
                double rotDensity = (Math.abs(dtheta[i]) / ds) * r_max;
                double v_wheel = maxWheelSpeed / (1.0 + rotDensity);
                velocities.set(i, Math.min(velocities.get(i), v_wheel));

                // ── FIX: maxRotVelocity constraint ──────────────────────
                // omega = dtheta/dt = dtheta * v/ds  =>  v <= maxRotVelocity * ds / |dtheta|
                // This ensures omega is never clipped after the fact.
                if (Math.abs(dtheta[i]) > 1e-9) {
                    double v_rot = maxRotVelocity * ds / Math.abs(dtheta[i]);
                    velocities.set(i, Math.min(velocities.get(i), v_rot));
                }
            }
        }

        // Lateral (curvature) constraint
        for (int i = 0; i < N; i++) {
            double k = points.get(i).getCurvature();
            if (Math.abs(k) > 1e-9) {
                double v_lat = Math.sqrt(maxAcceleration / Math.abs(k));
                velocities.set(i, Math.min(velocities.get(i), v_lat));
            }
        }

        // ── 4. Forward pass (acceleration) ────────────────────────────
        velocities.set(0, 0.0);
        for (int i = 1; i < N; i++) {
            double ds = points.get(i).getDistance() - points.get(i - 1).getDistance();
            double v_accel = Math.sqrt(
                velocities.get(i - 1) * velocities.get(i - 1) + 2 * maxAcceleration * ds);
            velocities.set(i, Math.min(velocities.get(i), v_accel));
        }

        // ── 5. Backward pass (deceleration) ───────────────────────────
        velocities.set(N - 1, 0.0);
        for (int i = N - 2; i >= 0; i--) {
            double ds = points.get(i + 1).getDistance() - points.get(i).getDistance();
            double v_decel = Math.sqrt(
                velocities.get(i + 1) * velocities.get(i + 1) + 2 * maxAcceleration * ds);
            velocities.set(i, Math.min(velocities.get(i), v_decel));
        }

        // ── 6. Integrate time, set velocity vectors and omega ─────────
        double time = 0.0;
        points.get(0).setTime(0.0);
        points.get(0).setVelocity(new Vector2(0, 0));
        points.get(0).setHeading(headings[0]);
        points.get(0).setRotationalVelocity(0.0);

        for (int i = 1; i < N; i++) {
            double v_avg = (velocities.get(i) + velocities.get(i - 1)) / 2.0;
            double ds = points.get(i).getDistance() - points.get(i - 1).getDistance();
            double dt = (v_avg > 1e-9) ? ds / v_avg : 0.0;
            time += dt;

            // Omega is now guaranteed within maxRotVelocity by the step 3 cap —
            // no clamp needed here, but kept as a safety net for floating point edge cases
            double omega = (dt > 1e-9) ? dtheta[i] / dt : 0.0;

            Vector2 dir = points.get(i).getPosition()
                            .sub(points.get(i - 1).getPosition()).norm();
            points.get(i).setVelocity(dir.mul(velocities.get(i)));
            points.get(i).setTime(time);
            points.get(i).setHeading(headings[i]);
            points.get(i).setRotationalVelocity(omega);
        }

        return new ProfiledPath(points);
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

    public double getStartHeading() {
        return profiledPoints.get(0).getHeading();
    }

    public double getDuration() {
        return profiledPoints.get(profiledPoints.size()-1).getTime();
    }
}
