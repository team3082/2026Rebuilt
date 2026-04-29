package frc.robot.utils.trajectories;

import java.util.List;
import frc.robot.utils.Vector2;

public class ProfiledPath {

    private final List<PathPoint> points;

    public ProfiledPath(List<PathPoint> points) {
        this.points = points;
    }

    public PathPoint getPointAtTime(double t) {
        if (points.size() == 1)
            return points.get(0).copy();
        int lo = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            if (points.get(i).getTime() <= t)
                lo = i;
            else
                break;
        }
        int hi = Math.min(lo + 1, points.size() - 1);
        if (lo == hi)
            return points.get(lo).copy();
        PathPoint p0 = points.get(lo), p1 = points.get(hi);
        double dt = p1.getTime() - p0.getTime();
        double a = dt > 1e-9 ? (t - p0.getTime()) / dt : 0.0;
        return interpolate(p0, p1, a);
    }

    public PathPoint getClosestPoint(Vector2 robotPos) {
        if (points.isEmpty())
            return null;
        if (points.size() == 1)
            return points.get(0).copy();
        double minDist = Double.POSITIVE_INFINITY;
        int bestIdx = 0;
        double bestA = 0;
        for (int i = 0; i < points.size() - 1; i++) {
            Vector2 a = points.get(i).getPosition(), b = points.get(i + 1).getPosition();
            Vector2 ab = b.sub(a), ar = robotPos.sub(a);
            double abLenSq = ab.x * ab.x + ab.y * ab.y;
            double alpha = abLenSq > 0 ? Math.max(0, Math.min(1, (ar.x * ab.x + ar.y * ab.y) / abLenSq)) : 0;
            double dist = a.add(ab.mul(alpha)).dist(robotPos);
            if (dist < minDist) {
                minDist = dist;
                bestIdx = i;
                bestA = alpha;
            }
        }
        return interpolate(points.get(bestIdx), points.get(bestIdx + 1), bestA);
    }

    public double getClosestPointDistance(Vector2 robotPos) {
        if (points.isEmpty())
            return 0;
        if (points.size() == 1)
            return points.get(0).getPosition().dist(robotPos);
        double minDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < points.size() - 1; i++) {
            Vector2 a = points.get(i).getPosition(), b = points.get(i + 1).getPosition();
            Vector2 ab = b.sub(a), ap = robotPos.sub(a);
            double abLenSq = ab.x * ab.x + ab.y * ab.y;
            double alpha = abLenSq > 0 ? Math.max(0, Math.min(1, (ap.x * ab.x + ap.y * ab.y) / abLenSq)) : 0;
            double dist = a.add(ab.mul(alpha)).dist(robotPos);
            if (dist < minDist)
                minDist = dist;
        }
        return minDist;
    }

    public Vector2 getStartPoint() {
        return points.get(0).getPosition();
    }

    public double getStartHeading() {
        return points.get(0).getHeading();
    }

    public double getDuration() {
        return points.get(points.size() - 1).getTime();
    }

    public List<PathPoint> getPoints() {
        return points;
    }

    private static PathPoint interpolate(PathPoint p0, PathPoint p1, double a) {
        PathPoint out = new PathPoint();
        out.position = p0.position.mul(1 - a).add(p1.position.mul(a));
        out.velocity = p0.velocity.mul(1 - a).add(p1.velocity.mul(a));
        out.curvature = lerp(p0.curvature, p1.curvature, a);
        out.acceleration = lerp(p0.acceleration, p1.acceleration, a);
        out.s = lerp(p0.s, p1.s, a);
        out.time = lerp(p0.time, p1.time, a);
        out.heading = lerpHeading(p0.heading, p1.heading, a);
        out.rotationalVelocity = lerp(p0.rotationalVelocity, p1.rotationalVelocity, a);
        return out;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double lerpHeading(double h0, double h1, double t) {
        double d = h1 - h0;
        while (d > Math.PI)
            d -= 2 * Math.PI;
        while (d < -Math.PI)
            d += 2 * Math.PI;
        return h0 + d * t;
    }
}
