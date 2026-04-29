package frc.robot.utils.trajectories;

import java.util.*;

import frc.robot.utils.Vector2;

public class Trajectory {

    private static final int OVERSAMPLING_FACTOR = 100;
    private static final double SAMPLING_DISTANCE = 1.0;
    static final double EPSILON = 1e-9;
    static final double FIELD_HEIGHT_INCHES = 317.69;

    // =========================================================================
    // Public types
    // =========================================================================

    public static class MotionSettings {
        public double maxTranslationalVelocity = 170.0;
        public double maxRotationalVelocity = 5.0;
        public double maxWheelSpeed = 170.0;
        public double maxAcceleration = 170.0;
        public double maxLateralAcceleration = 170.0;
        public double swerveRadius = 14.0;

        public MotionSettings sanitized() {
            double min = 0.001;
            MotionSettings s = new MotionSettings();
            s.maxTranslationalVelocity = Math.max(maxTranslationalVelocity, min);
            s.maxRotationalVelocity = Math.max(maxRotationalVelocity, min);
            s.maxWheelSpeed = Math.max(maxWheelSpeed, min);
            s.maxAcceleration = Math.max(maxAcceleration, min);
            s.maxLateralAcceleration = Math.max(maxLateralAcceleration, min);
            s.swerveRadius = Math.max(swerveRadius, min);
            return s;
        }
    }

    public static class AnchorPoint {
        public Vector2 position = new Vector2();
        public Vector2 handleInOffset = new Vector2();
        public Vector2 handleOutOffset = new Vector2();
        public boolean isCurved;
        public boolean handlesAligned;
        public String name = "";
    }

    /** Raw geometry point — x/y in field inches, no velocity/time profiling. */
    public static class TrajPoint {
        public double x, y;
        public double s;
        public double curvature;
        public Vector2 velocity = new Vector2();
        public double acceleration;
        public double time;
        public double heading;
        public double rotationalVelocity;

        public TrajPoint copy() {
            TrajPoint p = new TrajPoint();
            p.x = x;
            p.y = y;
            p.s = s;
            p.curvature = curvature;
            p.velocity = new Vector2(velocity.x, velocity.y);
            p.acceleration = acceleration;
            p.time = time;
            p.heading = heading;
            p.rotationalVelocity = rotationalVelocity;
            return p;
        }
    }

    public static class TrajectoryResult {
        public double totalTime;
        public List<TrajPoint> points = new ArrayList<>();
        public List<TrajPoint> geomPath = new ArrayList<>(); // raw geometry samples
        public double[] geomTs = new double[0]; // t value per geom sample
    }

    public enum ControlPointAttributeType {
        STOP, ROTATE, COMMAND, LOOP, MOTION_LIMITS
    }

    public static class ControlPointAttribute {
        public ControlPointAttributeType type;
        public double duration;
        public double heading;
        public boolean stopping;
        public int bounces;
        public int targetLoopId = -1;
        public double velocity;
        public double acceleration;
    }

    public static class ControlPoint {
        public long id;
        public double u;
        public String name = "";
        public String color = "";
        public List<ControlPointAttribute> attributes = new ArrayList<>();
    }

    public enum ActionKindType {
        STOP, ROTATE, COMMAND
    }

    public static class ActionDescriptor {
        public double t;
        public ActionKindType kind;
        public double stopDuration;
        public double rotateHeading;
        public boolean commandStopping;
    }

    public static class MotionLimitFrame {
        public double t;
        public double maxVelocity;
        public double maxAcceleration;
    }

    // =========================================================================
    // Public entry points
    // =========================================================================

    public static TrajectoryResult computeTravelTime(
            List<AnchorPoint> anchors,
            List<ControlPoint> controlPoints,
            MotionSettings settings) {
        return computeWithOrientation(anchors, controlPoints, settings, false);
    }

    public static List<ActionDescriptor> parseActionsPublic(List<ControlPoint> cps, int curveCount) {
        return parseActions(cps, curveCount);
    }

    public static List<Double> collectSplitValuesPublic(List<ActionDescriptor> actions) {
        return collectSplitValues(actions);
    }

    // =========================================================================
    // Core pipeline
    // =========================================================================

    static TrajectoryResult computeWithOrientation(
            List<AnchorPoint> anchors,
            List<ControlPoint> controlPoints,
            MotionSettings settings,
            boolean flipped) {

        if (anchors.size() < 2)
            return emptyResult();

        // Generate geometry samples — needed both for profiling and for segment
        // splitting
        GeomResult geom = generatePathPoints(anchors);
        if (geom.points.size() < 2)
            return emptyResult();

        if (controlPoints == null)
            controlPoints = Collections.emptyList();
        int curveCount = Math.max(anchors.size() - 1, 0);

        List<ActionDescriptor> actions = parseActions(controlPoints, curveCount);
        List<MotionLimitFrame> motionLimits = parseMotionLimits(controlPoints, curveCount);
        List<Double> splitValues = collectSplitValues(actions);
        List<double[]> rotByDist = buildRotateKeyframes(geom.points, geom.ts, actions, flipped);
        List<Object[]> segments = splitPath(geom.points, geom.ts, splitValues);

        List<TrajPoint> all = new ArrayList<>();
        double cumTime = 0;
        double distOffset = 0;

        for (int si = 0; si < segments.size(); si++) {
            List<TrajPoint> segPts = cast(segments.get(si)[0]);
            double[] segTs = (double[]) segments.get(si)[1];
            if (segPts.size() < 2)
                continue;

            List<Double> headings = buildTargetHeadings(segPts, rotByDist, distOffset);
            List<TrajPoint> profiled = profileSegment(segPts, segTs, headings, motionLimits, settings);
            if (profiled.isEmpty())
                continue;

            for (TrajPoint p : profiled)
                p.time += cumTime;

            if (!all.isEmpty()) {
                TrajPoint a = all.get(all.size() - 1), b = profiled.get(0);
                if (Math.sqrt(sq(a.x - b.x) + sq(a.y - b.y)) < EPSILON)
                    profiled.remove(0);
            }
            if (profiled.isEmpty())
                continue;

            cumTime = profiled.get(profiled.size() - 1).time;
            all.addAll(profiled);

            if (si < splitValues.size()) {
                double splitT = splitValues.get(si);
                double stopDur = stopDurationAtT(actions, splitT);
                if (stopDur > EPSILON && !all.isEmpty()) {
                    TrajPoint hold = all.get(all.size() - 1).copy();
                    Double stopH = rotateHeadingAtT(actions, splitT);
                    if (stopH != null) {
                        double delta = stopH - hold.heading;
                        while (delta > Math.PI)
                            delta -= 2 * Math.PI;
                        while (delta < -Math.PI)
                            delta += 2 * Math.PI;
                        hold.heading += delta;
                        hold.rotationalVelocity = delta / stopDur;
                    } else {
                        hold.rotationalVelocity = 0;
                    }
                    hold.time += stopDur;
                    hold.velocity = new Vector2();
                    hold.acceleration = 0;
                    all.add(hold);
                    cumTime += stopDur;
                }
            }

            double segLen = 0;
            for (int k = 0; k < segPts.size() - 1; k++)
                segLen += Math
                        .sqrt(sq(segPts.get(k + 1).x - segPts.get(k).x) + sq(segPts.get(k + 1).y - segPts.get(k).y));
            distOffset += segLen;
        }

        TrajectoryResult res = new TrajectoryResult();
        res.totalTime = all.isEmpty() ? 0 : all.get(all.size() - 1).time;
        res.points = all;
        res.geomPath = geom.points;
        res.geomTs = geom.ts;
        return res;
    }

    // =========================================================================
    // Path splitting (internal, for profiling)
    // =========================================================================

    private static List<Object[]> splitPath(List<TrajPoint> path, double[] ts, List<Double> splitValues) {
        if (path.size() < 2)
            return Collections.emptyList();
        if (ts.length != path.size() || splitValues.isEmpty())
            return Collections.singletonList(new Object[] { new ArrayList<>(path), ts });

        List<Double> bounds = new ArrayList<>(splitValues);
        bounds.replaceAll(v -> clamp01(v));
        bounds.sort(Double::compareTo);
        List<Double> db = new ArrayList<>();
        for (double b : bounds)
            if (db.isEmpty() || Math.abs(db.get(db.size() - 1) - b) >= EPSILON)
                db.add(b);
        db.add(1.0);

        List<Object[]> segs = new ArrayList<>();
        double startT = 0;
        for (double endT : db) {
            if (endT <= startT + EPSILON)
                continue;
            Object[] seg = buildSegment(path, ts, startT, endT);
            if (seg != null)
                segs.add(seg);
            startT = endT;
        }
        if (segs.isEmpty())
            segs.add(new Object[] { new ArrayList<>(path), ts });
        return segs;
    }

    private static Object[] buildSegment(List<TrajPoint> path, double[] ts, double tStart, double tEnd) {
        tStart = clamp01(tStart);
        tEnd = clamp01(tEnd);
        if (tEnd <= tStart + EPSILON)
            return null;
        List<TrajPoint> pts = new ArrayList<>();
        List<Double> tl = new ArrayList<>();
        pts.add(interpPointAtT(path, ts, tStart));
        tl.add(tStart);
        for (int i = 0; i < ts.length; i++)
            if (ts[i] > tStart + EPSILON && ts[i] < tEnd - EPSILON) {
                pts.add(path.get(i).copy());
                tl.add(ts[i]);
            }
        pts.add(interpPointAtT(path, ts, tEnd));
        tl.add(tEnd);
        if (pts.size() < 2)
            return null;
        double[] arr = new double[tl.size()];
        for (int i = 0; i < tl.size(); i++)
            arr[i] = tl.get(i);
        return new Object[] { pts, arr };
    }

    private static TrajPoint interpPointAtT(List<TrajPoint> path, double[] ts, double t) {
        if (path.isEmpty())
            return new TrajPoint();
        t = clamp01(t);
        if (t <= ts[0])
            return path.get(0).copy();
        int last = ts.length - 1;
        if (t >= ts[last])
            return path.get(last).copy();
        int idx = bsearch(ts, t);
        if (idx >= 0)
            return path.get(idx).copy();
        int ins = -(idx + 1), lo = Math.max(ins - 1, 0), hi = Math.min(ins, last);
        double span = Math.abs(ts[hi] - ts[lo]);
        double a = span <= EPSILON ? 0 : clamp01((t - ts[lo]) / (ts[hi] - ts[lo]));
        TrajPoint p0 = path.get(lo), p1 = path.get(hi), out = new TrajPoint();
        out.x = lerp(p0.x, p1.x, a);
        out.y = lerp(p0.y, p1.y, a);
        out.s = lerp(p0.s, p1.s, a);
        out.curvature = lerp(p0.curvature, p1.curvature, a);
        return out;
    }

    // =========================================================================
    // Geometry generation (public so FeatherFlow can call it for segment splitting)
    // =========================================================================

    public static class GeomResult {
        public List<TrajPoint> points;
        public double[] ts;
    }

    public static GeomResult generatePathPoints(List<AnchorPoint> anchors) {
        List<TrajPoint> path = new ArrayList<>();
        List<Double> tsOut = new ArrayList<>();
        double cumS = 0;
        int curveCount = Math.max(anchors.size() - 1, 1);

        for (int i = 0; i < anchors.size() - 1; i++) {
            AnchorPoint a0 = anchors.get(i), a1 = anchors.get(i + 1);
            BezierCurve curve = new BezierCurve(
                    a0.position, a0.position.add(a0.handleOutOffset),
                    a1.position.add(a1.handleInOffset), a1.position);

            double[][] lut = new double[OVERSAMPLING_FACTOR + 1][2];
            Vector2 last = curve.position(0);
            double sLocal = 0;
            lut[0][0] = 0;
            lut[0][1] = 0;
            for (int j = 1; j <= OVERSAMPLING_FACTOR; j++) {
                double tp = (double) j / OVERSAMPLING_FACTOR;
                Vector2 pos = curve.position(tp);
                sLocal += pos.sub(last).mag();
                lut[j][0] = sLocal;
                lut[j][1] = tp;
                last = pos;
            }
            double segLen = sLocal;

            double sAlong = 0;
            boolean first = path.isEmpty();
            while (sAlong <= segLen + 1e-9) {
                double t = interpTForDist(lut, sAlong);
                Vector2 pos = curve.position(t);
                if (!first && !path.isEmpty()) {
                    TrajPoint lp = path.get(path.size() - 1);
                    if (pos.sub(new Vector2(lp.x, lp.y)).mag() < 1e-9) {
                        sAlong += SAMPLING_DISTANCE;
                        continue;
                    }
                }
                TrajPoint pp = new TrajPoint();
                pp.x = pos.x;
                pp.y = pos.y;
                pp.s = cumS + sAlong;
                pp.curvature = curve.curvature(t);
                path.add(pp);
                tsOut.add(clamp01((i + clamp01(t)) / curveCount));
                sAlong += SAMPLING_DISTANCE;
                first = false;
            }
            cumS += segLen;
        }

        Vector2 lpos = anchors.get(anchors.size() - 1).position;
        if (!path.isEmpty()) {
            TrajPoint lp = path.get(path.size() - 1);
            double d = lpos.sub(new Vector2(lp.x, lp.y)).mag();
            if (d > 1e-9) {
                TrajPoint pp = new TrajPoint();
                pp.x = lpos.x;
                pp.y = lpos.y;
                pp.s = lp.s + d;
                path.add(pp);
                tsOut.add(1.0);
            }
        } else {
            TrajPoint pp = new TrajPoint();
            pp.x = lpos.x;
            pp.y = lpos.y;
            path.add(pp);
            tsOut.add(1.0);
        }

        annotateDiscreteCurvature(path);

        double[] tsArr = new double[tsOut.size()];
        for (int i = 0; i < tsOut.size(); i++)
            tsArr[i] = tsOut.get(i);

        GeomResult r = new GeomResult();
        r.points = path;
        r.ts = tsArr;
        return r;
    }

    private static void annotateDiscreteCurvature(List<TrajPoint> path) {
        if (path.size() < 3)
            return;
        double[] est = new double[path.size()];
        for (int i = 1; i < path.size() - 1; i++) {
            double ax = path.get(i).x - path.get(i - 1).x, ay = path.get(i).y - path.get(i - 1).y;
            double bx = path.get(i + 1).x - path.get(i).x, by = path.get(i + 1).y - path.get(i).y;
            double aL = Math.hypot(ax, ay), bL = Math.hypot(bx, by);
            double cL = Math.hypot(path.get(i + 1).x - path.get(i - 1).x, path.get(i + 1).y - path.get(i - 1).y);
            if (aL <= EPSILON || bL <= EPSILON || cL <= EPSILON)
                continue;
            est[i] = (2 * Math.abs(ax * by - ay * bx)) / (aL * bL * cL);
        }
        for (int i = 0; i < path.size(); i++) {
            TrajPoint p = path.get(i);
            if (est[i] > Math.abs(p.curvature))
                p.curvature = (p.curvature == 0 ? 1 : Math.signum(p.curvature)) * est[i];
        }
    }

    // =========================================================================
    // Profiler
    // =========================================================================

    private static List<TrajPoint> profileSegment(
            List<TrajPoint> seg, double[] segTs, List<Double> targetH,
            List<MotionLimitFrame> limits, MotionSettings ms) {

        if (seg.size() < 2)
            return Collections.emptyList();
        int n = seg.size();
        List<TrajPoint> pts = new ArrayList<>();
        for (TrajPoint p : seg) {
            TrajPoint np = new TrajPoint();
            np.x = p.x;
            np.y = p.y;
            np.s = p.s;
            np.curvature = p.curvature;
            pts.add(np);
        }

        double[] dist = new double[n];
        for (int i = 1; i < n; i++)
            dist[i] = dist[i - 1]
                    + Math.sqrt(sq(pts.get(i).x - pts.get(i - 1).x) + sq(pts.get(i).y - pts.get(i - 1).y));
        double totalDist = dist[n - 1];

        double[] maxV = new double[n], maxA = new double[n];
        for (int i = 0; i < n; i++) {
            double rel = totalDist > EPSILON ? dist[i] / totalDist : 0;
            double t = (segTs != null && i < segTs.length) ? clamp01(segTs[i]) : rel;
            double[] va = resolveMotionLimit(limits, t, ms);
            maxV[i] = va[0];
            maxA[i] = va[1];
        }

        double[] headings = new double[n];
        if (targetH.size() == n)
            for (int i = 0; i < n; i++)
                headings[i] = targetH.get(i);
        for (int i = 1; i < n; i++) {
            double d = headings[i] - headings[i - 1];
            while (d > Math.PI)
                d -= 2 * Math.PI;
            while (d < -Math.PI)
                d += 2 * Math.PI;
            headings[i] = headings[i - 1] + d;
        }
        double[] dtheta = new double[n];
        for (int i = 1; i < n; i++)
            dtheta[i] = headings[i] - headings[i - 1];

        double[] vel = Arrays.copyOf(maxV, n);
        for (int i = 1; i < n; i++) {
            double ds = dist[i] - dist[i - 1];
            if (ds <= EPSILON)
                continue;
            vel[i] = Math.min(vel[i], ms.maxWheelSpeed / (1 + (Math.abs(dtheta[i]) / ds) * ms.swerveRadius));
            if (Math.abs(dtheta[i]) > EPSILON)
                vel[i] = Math.min(vel[i], ms.maxRotationalVelocity * ds / Math.abs(dtheta[i]));
        }
        for (int i = 0; i < n; i++) {
            double k = pts.get(i).curvature;
            if (Math.abs(k) > EPSILON)
                vel[i] = Math.min(vel[i], Math.sqrt(Math.min(ms.maxLateralAcceleration, maxA[i]) / Math.abs(k)));
        }
        vel[0] = 0;
        for (int i = 1; i < n; i++) {
            double ds = Math.max(dist[i] - dist[i - 1], 0);
            vel[i] = Math.min(vel[i], Math.sqrt(vel[i - 1] * vel[i - 1] + 2 * Math.min(maxA[i - 1], maxA[i]) * ds));
        }
        vel[n - 1] = 0;
        for (int i = n - 2; i >= 0; i--) {
            double ds = Math.max(dist[i + 1] - dist[i], 0);
            vel[i] = Math.min(vel[i], Math.sqrt(vel[i + 1] * vel[i + 1] + 2 * Math.min(maxA[i], maxA[i + 1]) * ds));
        }

        pts.get(0).heading = headings[0];
        double time = 0;
        for (int i = 1; i < n; i++) {
            double ds = dist[i] - dist[i - 1], vAvg = (vel[i] + vel[i - 1]) / 2, dt = vAvg > EPSILON ? ds / vAvg : 0;
            time += dt;
            TrajPoint p = pts.get(i);
            p.velocity = new Vector2(pts.get(i).x - pts.get(i - 1).x, pts.get(i).y - pts.get(i - 1).y).norm()
                    .mul(vel[i]);
            p.time = time;
            p.heading = headings[i];
            p.rotationalVelocity = dt > EPSILON ? dtheta[i] / dt : 0;
            p.acceleration = dt > EPSILON ? (vel[i] - vel[i - 1]) / dt : 0;
        }
        return pts;
    }

    // =========================================================================
    // Heading helpers
    // =========================================================================

    private static List<double[]> buildRotateKeyframes(List<TrajPoint> path, double[] ts,
            List<ActionDescriptor> actions, boolean flipped) {
        if (path.size() < 2)
            return Collections.emptyList();
        List<double[]> ktH = new ArrayList<>();
        for (ActionDescriptor a : actions) {
            if (a.kind != ActionKindType.ROTATE)
                continue;
            double rad = flipped ? Math.toRadians((360.0 - a.rotateHeading) + 180.0) : Math.toRadians(a.rotateHeading);
            ktH.add(new double[] { clamp01(a.t), rad });
        }
        if (ktH.isEmpty())
            return Collections.emptyList();
        ktH.sort(Comparator.comparingDouble(e -> e[0]));
        List<double[]> byDist = new ArrayList<>();
        for (double[] kf : ktH)
            byDist.add(new double[] { interpDistAtT(path, ts, kf[0]), kf[1] });
        byDist.sort(Comparator.comparingDouble(e -> e[0]));
        List<double[]> dd = new ArrayList<>();
        for (double[] e : byDist) {
            if (!dd.isEmpty() && Math.abs(dd.get(dd.size() - 1)[0] - e[0]) < EPSILON) {
                dd.get(dd.size() - 1)[1] = e[1];
                continue;
            }
            dd.add(e);
        }
        return dd;
    }

    private static List<Double> buildTargetHeadings(List<TrajPoint> seg, List<double[]> rbd, double off) {
        if (seg.isEmpty())
            return Collections.emptyList();
        if (rbd.isEmpty())
            return new ArrayList<>(Collections.nCopies(seg.size(), 0.0));
        double[] d = new double[seg.size()];
        for (int i = 1; i < seg.size(); i++)
            d[i] = d[i - 1] + Math.sqrt(sq(seg.get(i).x - seg.get(i - 1).x) + sq(seg.get(i).y - seg.get(i - 1).y));
        List<Double> h = new ArrayList<>();
        for (double di : d)
            h.add(interpHeadingByDist(rbd, off + di));
        return h;
    }

    private static double interpHeadingByDist(List<double[]> rbd, double d) {
        if (rbd.isEmpty())
            return 0;
        if (d <= rbd.get(0)[0])
            return rbd.get(0)[1];
        double[] last = rbd.get(rbd.size() - 1);
        if (d >= last[0])
            return last[1];
        for (int k = 0; k < rbd.size() - 1; k++) {
            double pd = rbd.get(k)[0], ph = rbd.get(k)[1], nd = rbd.get(k + 1)[0], nh = rbd.get(k + 1)[1];
            if (pd <= d && d < nd) {
                double span = nd - pd, a = span > EPSILON ? (d - pd) / span : 0, delta = nh - ph;
                while (delta > Math.PI)
                    delta -= 2 * Math.PI;
                while (delta < -Math.PI)
                    delta += 2 * Math.PI;
                return ph + a * delta;
            }
        }
        return last[1];
    }

    // =========================================================================
    // Control parsing
    // =========================================================================

    private static List<ActionDescriptor> parseActions(List<ControlPoint> cps, int curveCount) {
        List<ActionDescriptor> actions = new ArrayList<>();
        int cc = Math.max(curveCount, 1);
        for (ControlPoint cp : cps) {
            double gt = normalizeU(cp.u, cc);
            for (ControlPointAttribute attr : cp.attributes) {
                ActionDescriptor a = new ActionDescriptor();
                a.t = gt;
                switch (attr.type) {
                    case STOP:
                        a.kind = ActionKindType.STOP;
                        a.stopDuration = Math.max(attr.duration, 0);
                        break;
                    case ROTATE:
                        a.kind = ActionKindType.ROTATE;
                        a.rotateHeading = attr.heading;
                        break;
                    case COMMAND:
                        a.kind = ActionKindType.COMMAND;
                        a.commandStopping = attr.stopping;
                        break;
                    default:
                        continue;
                }
                actions.add(a);
            }
        }
        actions.sort(Comparator.comparingDouble(a -> a.t));
        return actions;
    }

    private static List<MotionLimitFrame> parseMotionLimits(List<ControlPoint> cps, int curveCount) {
        int cc = Math.max(curveCount, 1);
        List<MotionLimitFrame> limits = new ArrayList<>();
        for (ControlPoint cp : cps) {
            double gt = normalizeU(cp.u, cc);
            for (ControlPointAttribute attr : cp.attributes) {
                if (attr.type != ControlPointAttributeType.MOTION_LIMITS || attr.velocity <= EPSILON
                        || attr.acceleration <= EPSILON)
                    continue;
                MotionLimitFrame f = new MotionLimitFrame();
                f.t = gt;
                f.maxVelocity = attr.velocity;
                f.maxAcceleration = attr.acceleration;
                limits.add(f);
            }
        }
        limits.sort(Comparator.comparingDouble(f -> f.t));
        List<MotionLimitFrame> dd = new ArrayList<>();
        for (MotionLimitFrame f : limits) {
            if (!dd.isEmpty() && Math.abs(dd.get(dd.size() - 1).t - f.t) < EPSILON) {
                dd.get(dd.size() - 1).maxVelocity = Math.min(dd.get(dd.size() - 1).maxVelocity, f.maxVelocity);
                dd.get(dd.size() - 1).maxAcceleration = Math.min(dd.get(dd.size() - 1).maxAcceleration,
                        f.maxAcceleration);
                continue;
            }
            dd.add(f);
        }
        return dd;
    }

    private static List<Double> collectSplitValues(List<ActionDescriptor> actions) {
        List<Double> splits = new ArrayList<>();
        for (ActionDescriptor a : actions) {
            if (a.kind == ActionKindType.STOP)
                splits.add(a.t);
            else if (a.kind == ActionKindType.COMMAND && a.commandStopping)
                splits.add(a.t);
        }
        splits.sort(Double::compareTo);
        List<Double> dd = new ArrayList<>();
        for (double v : splits)
            if (dd.isEmpty() || Math.abs(dd.get(dd.size() - 1) - v) >= EPSILON)
                dd.add(v);
        return dd;
    }

    private static double stopDurationAtT(List<ActionDescriptor> actions, double t) {
        double total = 0;
        for (ActionDescriptor a : actions)
            if (a.kind == ActionKindType.STOP && Math.abs(a.t - t) < 1e-6)
                total += a.stopDuration;
        return total;
    }

    private static Double rotateHeadingAtT(List<ActionDescriptor> actions, double t) {
        Double r = null;
        for (ActionDescriptor a : actions)
            if (a.kind == ActionKindType.ROTATE && Math.abs(a.t - t) < 1e-6)
                r = Math.toRadians(a.rotateHeading);
        return r;
    }

    private static double[] resolveMotionLimit(List<MotionLimitFrame> frames, double t, MotionSettings ms) {
        double v = ms.maxTranslationalVelocity, a = ms.maxAcceleration;
        for (MotionLimitFrame f : frames) {
            if (f.t <= t + EPSILON) {
                v = f.maxVelocity;
                a = f.maxAcceleration;
            } else
                break;
        }
        return new double[] { v, a };
    }

    // =========================================================================
    // BezierCurve
    // =========================================================================

    private static class BezierCurve {
        final Vector2 p0, p1, p2, p3;

        BezierCurve(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) {
            this.p0 = p0;
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
        }

        Vector2 position(double t) {
            double ti = 1 - t, ti2 = ti * ti, t2 = t * t;
            return p0.mul(ti2 * ti).add(p1.mul(3 * ti2 * t)).add(p2.mul(3 * ti * t2)).add(p3.mul(t2 * t));
        }

        Vector2 derivative(double t) {
            double ti = 1 - t;
            return p1.sub(p0).mul(3 * ti * ti).add(p2.sub(p1).mul(6 * ti * t)).add(p3.sub(p2).mul(3 * t * t));
        }

        Vector2 secondDerivative(double t) {
            return p2.sub(p1.mul(2)).add(p0).mul(6 * (1 - t)).add(p3.sub(p2.mul(2)).add(p1).mul(6 * t));
        }

        double curvature(double t) {
            Vector2 d = derivative(t), dd = secondDerivative(t);
            double den = Math.pow(d.mag(), 3);
            return Math.abs(den) < 1e-9 ? 0 : (d.x * dd.y - d.y * dd.x) / den;
        }
    }

    // =========================================================================
    // Utilities
    // =========================================================================

    private static double normalizeU(double u, int cc) {
        double ci = Math.min(Math.max(Math.floor(u), 0), cc - 1.0);
        return clamp01((ci + clamp01(u - ci)) / cc);
    }

    public static double interpDistAtT(List<TrajPoint> path, double[] ts, double t) {
        if (path.isEmpty() || ts.length == 0)
            return 0;
        t = clamp01(t);
        if (t <= ts[0])
            return path.get(0).s;
        int last = ts.length - 1;
        if (t >= ts[last])
            return path.get(last).s;
        int idx = bsearch(ts, t);
        if (idx >= 0)
            return path.get(idx).s;
        int ins = -(idx + 1), lo = Math.max(ins - 1, 0), hi = Math.min(ins, last);
        double span = Math.abs(ts[hi] - ts[lo]);
        return lerp(path.get(lo).s, path.get(hi).s, span <= EPSILON ? 0 : clamp01((t - ts[lo]) / span));
    }

    private static double interpTForDist(double[][] lut, double dist) {
        if (dist <= 0)
            return 0;
        if (dist >= lut[lut.length - 1][0])
            return lut[lut.length - 1][1];
        int lo = 0, hi = lut.length - 1;
        while (lo < hi - 1) {
            int mid = (lo + hi) >>> 1;
            if (lut[mid][0] <= dist)
                lo = mid;
            else
                hi = mid;
        }
        return lut[lo][1] + (dist - lut[lo][0]) / (lut[hi][0] - lut[lo][0]) * (lut[hi][1] - lut[lo][1]);
    }

    private static int bsearch(double[] arr, double key) {
        int lo = 0, hi = arr.length - 1;
        while (lo <= hi) {
            int mid = (lo + hi) >>> 1;
            if (arr[mid] < key - EPSILON)
                lo = mid + 1;
            else if (arr[mid] > key + EPSILON)
                hi = mid - 1;
            else
                return mid;
        }
        return -(lo + 1);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp01(double v) {
        return Math.min(Math.max(v, 0), 1);
    }

    private static double sq(double v) {
        return v * v;
    }

    @SuppressWarnings("unchecked")
    private static List<TrajPoint> cast(Object o) {
        return (List<TrajPoint>) o;
    }

    private static TrajectoryResult emptyResult() {
        TrajectoryResult r = new TrajectoryResult();
        r.totalTime = 0;
        return r;
    }
}
