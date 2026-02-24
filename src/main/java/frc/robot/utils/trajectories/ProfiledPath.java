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

    public static ProfiledPath generateProfiledPath(RobotPath path, double maxVelocity, double maxAcceleration, double maxLateralAcceleration) {
        // Delegate with tangent-following headings (no rotation override)
        return generateProfiledPath(path, maxVelocity, maxAcceleration, maxLateralAcceleration, null);
    }

    /**
     * Generates a speed profile that jointly limits translational and rotational velocity
     * against the physical swerve module speed limit.
     *
     * <p>At each point the rotational demand consumes {@code |omega| * r_max} of module
     * bandwidth, leaving {@code v_module_max - |omega| * r_max} for translation.
     * Both translation and rotation are then forward/backward pass profiled against
     * {@code maxAcceleration}.
     *
     * @param path                  The geometric path to follow.
     * @param maxVelocity           Absolute translational speed cap (in/s).
     * @param maxAcceleration       Max translational + rotational accel (in/s² / rad/s²).
     * @param maxLateralAcceleration Max lateral acceleration for curvature limiting (in/s²).
     * @param targetHeadings        Per-point target headings in radians, or {@code null}
     *                              to follow the path tangent.
     */
    public static ProfiledPath generateProfiledPath(
            RobotPath path, double maxVelocity, double maxAcceleration,
            double maxLateralAcceleration, double[] targetHeadings) {

        if (path.getPoints().size() < 2) {
            return new ProfiledPath(new ArrayList<>());
        }

        ArrayList<ProfiledPoint> profiledPoints = initializeProfiledPoints(path);
        int N = profiledPoints.size();

        // ── 1. Resolve target headings ────────────────────────────────────────────
        double[] headings = new double[N];
        if (targetHeadings != null && targetHeadings.length == N) {
            // Caller supplied per-point headings (e.g. from rotate descriptors)
            System.arraycopy(targetHeadings, 0, headings, 0, N);
        } else {
            // Default: follow the path tangent
            headings[0] = profiledPoints.get(1).getPosition()
                              .sub(profiledPoints.get(0).getPosition()).atan2();
            for (int i = 1; i < N - 1; i++) {
                Vector2 dir = profiledPoints.get(i + 1).getPosition()
                                  .sub(profiledPoints.get(i - 1).getPosition());
                headings[i] = (dir.mag() > 1e-9) ? dir.atan2() : headings[i - 1];
            }
            headings[N - 1] = headings[N - 2];
        }

        // Unwrap headings to remove ±2π jumps
        for (int i = 1; i < N; i++) {
            double delta = headings[i] - headings[i - 1];
            while (delta >  Math.PI) delta -= 2 * Math.PI;
            while (delta < -Math.PI) delta += 2 * Math.PI;
            headings[i] = headings[i - 1] + delta;
        }

        // ── 2. Compute per-point heading change per unit distance (rad/in) ────────
        // This gives us the "rotational demand density" at each point.
        // r_max is the distance from center to the farthest module (in).
        double r_max = Math.hypot(
            Math.abs(Constants.Swerve.SWERVEMODX0),
            Math.abs(Constants.Swerve.SWERVEMODY0)
        );
        double v_module_max = Constants.Swerve.maxChassisVelocity;

        // dtheta[i] = heading change between point i-1 and i (radians)
        double[] dtheta = new double[N];
        dtheta[0] = 0.0;
        for (int i = 1; i < N; i++) {
            dtheta[i] = headings[i] - headings[i - 1];
        }

        // ── 3. Translational velocity ceiling at each point ───────────────────────
        // For a swerve drive the worst-case module speed is v_trans + |omega| * r_max.
        // We don't yet know v_trans (circular dependency), so we iteratively cap:
        // given ds between points, omega = dtheta / dt and dt = ds / v_trans →
        // omega = dtheta * v_trans / ds  →  v_trans + (dtheta/ds) * r_max * v_trans ≤ v_module_max
        // → v_trans ≤ v_module_max / (1 + (dtheta/ds) * r_max)
        ArrayList<Double> scalarVelocities = new ArrayList<>();
        for (int i = 0; i < N; i++) scalarVelocities.add(maxVelocity);

        // Lateral (curvature) constraint
        for (int i = 0; i < N; i++) {
            double curvature = profiledPoints.get(i).getCurvature();
            if (Math.abs(curvature) > 1e-9) {
                scalarVelocities.set(i, Math.min(scalarVelocities.get(i),
                    Math.sqrt(maxLateralAcceleration / Math.abs(curvature))));
            }
        }

        // Module-speed constraint: v_trans * (1 + dtheta/ds * r_max) ≤ v_module_max
        for (int i = 1; i < N; i++) {
            double ds = profiledPoints.get(i).getDistance() - profiledPoints.get(i - 1).getDistance();
            if (ds > 1e-9 && dtheta[i] > 1e-9) {
                double rotDemandFraction = (Math.abs(dtheta[i]) / ds) * r_max;
                double v_cap = v_module_max / (1.0 + rotDemandFraction);
                scalarVelocities.set(i, Math.min(scalarVelocities.get(i), v_cap));
            }
        }

        // ── 4. Forward pass (acceleration) ───────────────────────────────────────
        scalarVelocities.set(0, 0.0);
        for (int i = 1; i < N; i++) {
            double prev_v = scalarVelocities.get(i - 1);
            double ds = profiledPoints.get(i).getDistance() - profiledPoints.get(i - 1).getDistance();
            double v_accel = Math.sqrt(prev_v * prev_v + 2 * maxAcceleration * ds);
            scalarVelocities.set(i, Math.min(scalarVelocities.get(i), v_accel));
        }

        // ── 5. Backward pass (deceleration) ──────────────────────────────────────
        scalarVelocities.set(N - 1, 0.0);
        for (int i = N - 2; i >= 0; i--) {
            double next_v = scalarVelocities.get(i + 1);
            double ds = profiledPoints.get(i + 1).getDistance() - profiledPoints.get(i).getDistance();
            double v_decel = Math.sqrt(next_v * next_v + 2 * maxAcceleration * ds);
            scalarVelocities.set(i, Math.min(scalarVelocities.get(i), v_decel));
        }

        // ── 6. Integrate time, build velocity vectors and rotational velocity ─────
        double accumulatedTime = 0.0;
        profiledPoints.get(0).setTime(0.0);
        profiledPoints.get(0).setVelocity(new Vector2(0, 0));
        profiledPoints.get(0).setHeading(headings[0]);
        profiledPoints.get(0).setRotationalVelocity(0.0);

        for (int i = 1; i < N; i++) {
            double v_avg = (scalarVelocities.get(i) + scalarVelocities.get(i - 1)) / 2.0;
            double ds = profiledPoints.get(i).getDistance() - profiledPoints.get(i - 1).getDistance();
            double dt = (Math.abs(v_avg) > 1e-9) ? ds / v_avg : 0.0;
            accumulatedTime += dt;
            profiledPoints.get(i).setTime(accumulatedTime);

            // Translational velocity vector along path tangent
            Vector2 direction = profiledPoints.get(i).getPosition()
                                    .sub(profiledPoints.get(i - 1).getPosition()).norm();
            if (direction.mag() == 0 && i > 1) {
                direction = profiledPoints.get(i - 1).getPosition()
                                .sub(profiledPoints.get(i - 2).getPosition()).norm();
            }
            profiledPoints.get(i).setVelocity(direction.mul(scalarVelocities.get(i)).rotate(-Math.PI/2));

            // Rotational velocity: heading change over elapsed time
            double rotVel = (dt > 1e-9) ? (dtheta[i] / dt) : 0.0;
            profiledPoints.get(i).setHeading(headings[i]);
            profiledPoints.get(i).setRotationalVelocity(rotVel);
        }

        // ── 7. Translational acceleration ─────────────────────────────────────────
        for (int i = 1; i < N; i++) {
            double dv = scalarVelocities.get(i) - scalarVelocities.get(i - 1);
            double dt = profiledPoints.get(i).getTime() - profiledPoints.get(i - 1).getTime();
            profiledPoints.get(i).setAcceleration((dt > 1e-9) ? dv / dt : 0.0);
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
