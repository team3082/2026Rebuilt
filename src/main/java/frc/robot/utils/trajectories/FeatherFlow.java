package frc.robot.utils.trajectories;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.auto.commands.FollowPath;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.FeatherPath.FeatherActionDescriptor;
import frc.robot.utils.trajectories.FeatherPathDefinition.AnchorPoint;
import frc.robot.utils.trajectories.FeatherPathDefinition.ControlPoint;
import frc.robot.utils.trajectories.FeatherPathDefinition.ControlPointAttribute;

import java.io.File;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FeatherFlow {

    private static final Map<String, FeatherPath[]> trajectories = new HashMap<>();
    // If a global config.json exists in the FeatherFlow deploy dir, its
    // motionSettings
    // will be loaded here and preferred over per-path settings.
    private static volatile frc.robot.utils.trajectories.FeatherPathDefinition.MotionSettings GLOBAL_MOTION_SETTINGS = null;
    private static final ObjectMapper GLOBAL_MAPPER = new ObjectMapper();

    public static void init() {
        File directory = new File(Filesystem.getDeployDirectory(), "FeatherFlow");
        // Attempt to load global config.json (optional). If present, use its
        // motionSettings
        // as the global override for path motion settings.
        try {
            File cfg = new File(directory, "config.json");
            if (cfg.exists() && cfg.isFile()) {
                JsonNode root = GLOBAL_MAPPER.readTree(cfg);
                JsonNode motion = root.get("motionSettings");
                if (motion != null && !motion.isNull()) {
                    GLOBAL_MOTION_SETTINGS = GLOBAL_MAPPER.treeToValue(
                            motion, frc.robot.utils.trajectories.FeatherPathDefinition.MotionSettings.class);
                    System.out.println("[FeatherFlow] Loaded global motionSettings from config.json");
                } else {
                    System.out.println("[FeatherFlow] config.json present but contains no motionSettings");
                }
            }
        } catch (Exception e) {
            System.err.println("[FeatherFlow] Failed to read FeatherFlow/config.json: " + e.getMessage());
            // keep going with defaults
        }
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".ff"));
        if (files == null) {
            System.err.println("[FeatherFlow] Directory not found or empty: " + directory);
            return;
        }
        new Thread(() -> {
            for (File file : files) {
                String key = file.getName().replace(".ff", "");
                try {
                    FeatherPathDefinition def = FeatherPathDefinition.fromFile(file);
                    FeatherPath[] pair = new FeatherPath[2];
                    pair[0] = buildFeatherPath(def, false);
                    pair[1] = buildFeatherPath(def, true);
                    trajectories.put(key, pair);
                    System.out.println("[FeatherFlow] Loaded: " + file.getName()
                            + " — " + pair[0].paths.size() + " segment(s)");
                } catch (Exception e) {
                    System.err.println("[FeatherFlow] Failed to load: " + file.getName());
                    e.printStackTrace();
                }
            }
            System.out.println("[FeatherFlow] Done — " + trajectories.size() + " path(s) loaded.");
        }, "FeatherFlow-Loader").start();
    }

    public static FeatherPath getPath(String name, boolean flipped) {
        if (!trajectories.containsKey(name))
            throw new IllegalArgumentException(
                    "[FeatherFlow] Path '" + name + "' not found. Available: " + trajectories.keySet());
        return trajectories.get(name)[flipped ? 1 : 0];
    }

    public static SequentialCommandGroup buildFeatherAuto(String name, Command... commands) {
        return buildFeatherAuto(name, false, true, commands);
    }

    public static SequentialCommandGroup buildFeatherAuto(
            String name, boolean flipped, boolean resetOdo, Command... commands) {

        FeatherPath featherPath = getPath(name, flipped);
        SequentialCommandGroup group = new SequentialCommandGroup();

        if (resetOdo) {
            group.addCommands(
                    new InstantCommand(() -> SwervePosition.setPosition(featherPath.paths.get(0).getStartPoint())));
        }

        int commandIndex = 0;

        for (int pathIndex = 0; pathIndex < featherPath.paths.size(); pathIndex++) {
            ProfiledPath currentPath = featherPath.paths.get(pathIndex);

            double segmentStartTime = pathIndex == 0 ? 0.0 : getSegmentStartTime(featherPath, pathIndex);
            double segmentEndTime = getSegmentEndTime(featherPath, pathIndex, featherPath.totalTime);

            List<FeatherEvent> eventsForSegment = new ArrayList<>();
            for (FeatherActionDescriptor action : featherPath.actions) {
                if (!action.type.equals("command") || action.stopping)
                    continue;
                if (action.time >= segmentStartTime && action.time < segmentEndTime) {
                    double norm = (segmentEndTime - segmentStartTime) > 1e-9
                            ? (action.time - segmentStartTime) / (segmentEndTime - segmentStartTime)
                            : 0.5;
                    if (commandIndex < commands.length)
                        eventsForSegment.add(new FeatherEvent(norm, commands[commandIndex++]));
                }
            }

            group.addCommands(new InstantCommand(() -> System.out.println("[FeatherFlow] Following segment")));
            group.addCommands(new FollowPath(currentPath, eventsForSegment.toArray(new FeatherEvent[0])));

            for (FeatherActionDescriptor action : featherPath.actions) {
                if (Math.abs(action.time - segmentEndTime) > 0.005)
                    continue;
                if (action.type.equals("stop")) {
                    group.addCommands(new WaitCommand(action.duration));
                } else if (action.type.equals("command") && action.stopping) {
                    if (commandIndex < commands.length)
                        group.addCommands(commands[commandIndex++]);
                }
            }
        }
        return group;
    }

    // =========================================================================
    // Build
    // =========================================================================

    private static FeatherPath buildFeatherPath(FeatherPathDefinition def, boolean flipped) {
        List<Trajectory.AnchorPoint> anchors = convertAnchors(def.anchors, flipped);
        List<Trajectory.ControlPoint> cps = convertControlPoints(def.controlPoints);
        Trajectory.MotionSettings settings = convertMotionSettings(def.motionSettings);

        //flip rotations
        if (flipped){
            for (Trajectory.ControlPoint controlPoint : cps){
                for(Trajectory.ControlPointAttribute attribute : controlPoint.attributes){
                    if(attribute.type == Trajectory.ControlPointAttributeType.ROTATE){
                        //invert rotation
                        attribute.heading = -attribute.heading + 180;
                    }
                }
            }
        }

        Trajectory.TrajectoryResult result = Trajectory.computeTravelTime(anchors, cps, settings);

        if (result.points.isEmpty())
            return new FeatherPath(new ArrayList<>(), new ArrayList<>(), 0.0);

        int curveCount = Math.max(def.anchors.size() - 1, 0);
        List<Trajectory.ActionDescriptor> actions = Trajectory.parseActionsPublic(cps, curveCount);
        List<Double> splitValues = Trajectory.collectSplitValuesPublic(actions);

        // Port of Rust build_compiled_segments:
        // Use the geometry path (raw samples) to convert split t → arc-length s,
        // then find the index in the profiled path by s.
        List<ProfiledPath> segments = buildSegments(result, splitValues);
        List<FeatherActionDescriptor> featherActions = buildFeatherActions(actions, result);

        return new FeatherPath(segments, featherActions, result.totalTime);
    }

    /**
     * Exact port of Rust build_compiled_segments.
     *
     * 1. For each split t, convert t → arc-length s using the geometry path.
     * 2. Find the first profiled point index where point.s >= that target s.
     * 3. Slice the profiled point list between consecutive boundaries.
     * 4. Reset time to segment-relative.
     */
    private static List<ProfiledPath> buildSegments(
            Trajectory.TrajectoryResult result, List<Double> splitTs) {

        List<Trajectory.TrajPoint> profiled = result.points;
        List<Trajectory.TrajPoint> geomPath = result.geomPath;
        double[] geomTs = result.geomTs;

        List<ProfiledPath> segments = new ArrayList<>();
        if (profiled.size() < 2)
            return segments;

        // Build boundary indices — port of Rust segment_boundaries vec
        List<Integer> boundaries = new ArrayList<>();
        boundaries.add(0);

        for (double splitT : splitTs) {
            // interpolate_distance_at_t: convert split t → s using geometry samples
            double targetS = Trajectory.interpDistAtT(geomPath, geomTs, splitT);
            // find_point_index_at_s: first index where profiled[i].s >= targetS
            int idx = findPointIndexAtS(profiled, targetS);
            if (idx >= 0 && idx > boundaries.get(boundaries.size() - 1))
                boundaries.add(idx);
        }

        if (boundaries.get(boundaries.size() - 1) < profiled.size() - 1)
            boundaries.add(profiled.size() - 1);

        // Slice and convert each window
        for (int b = 0; b < boundaries.size() - 1; b++) {
            int startIdx = boundaries.get(b);
            int endIdx = boundaries.get(b + 1);
            if (startIdx >= endIdx)
                continue;

            List<Trajectory.TrajPoint> slice = profiled.subList(startIdx, endIdx + 1);
            if (slice.size() < 2)
                continue;

            double segStartTime = slice.get(0).time;
            List<PathPoint> out = new ArrayList<>(slice.size());
            for (Trajectory.TrajPoint ep : slice) {
                PathPoint pp = toPathPoint(ep, segStartTime);
                out.add(pp);
            }
            segments.add(new ProfiledPath(out));
        }

        return segments;
    }

    /** First index i where profiled[i].s >= targetS. Returns -1 if none. */
    private static int findPointIndexAtS(List<Trajectory.TrajPoint> pts, double targetS) {
        for (int i = 0; i < pts.size(); i++)
            if (pts.get(i).s >= targetS)
                return i;
        return -1;
    }

    /**
     * Converts one engine TrajPoint to a robot PathPoint.
     * - Inches → meters
     * - Field origin (bottom-left) → robot-centered (field center)
     * - Time reset to segment-relative
     */
    private static PathPoint toPathPoint(Trajectory.TrajPoint ep, double segStartTime) {
        PathPoint pp = new PathPoint();
        double rx = ep.x - (Constants.FIELD_WIDTH / 2.0);
        double ry = ep.y - (Constants.FIELD_HEIGHT / 2.0);
        pp.position = new frc.robot.utils.Vector2(rx, ry);
        pp.velocity = new frc.robot.utils.Vector2(ep.velocity.x, ep.velocity.y);
        pp.curvature = ep.curvature;
        pp.acceleration = ep.acceleration * 0.0254;
        pp.s = ep.s * 0.0254;
        pp.time = ep.time - segStartTime;
        pp.heading = ep.heading;
        pp.rotationalVelocity = ep.rotationalVelocity;
        return pp;
    }

    // =========================================================================
    // Action resolution
    // =========================================================================

    private static List<FeatherActionDescriptor> buildFeatherActions(
            List<Trajectory.ActionDescriptor> actions, Trajectory.TrajectoryResult result) {

        List<FeatherActionDescriptor> out = new ArrayList<>();
        double totalS = result.points.isEmpty() ? 1.0 : result.points.get(result.points.size() - 1).s;

        for (Trajectory.ActionDescriptor action : actions) {
            double targetS = action.t * totalS;
            double absTime = interpTimeAtS(result.points, targetS);

            FeatherActionDescriptor fd = new FeatherActionDescriptor();
            fd.t = action.t;
            fd.time = absTime;
            switch (action.kind) {
                case STOP:
                    fd.type = "stop";
                    fd.duration = action.stopDuration;
                    break;
                case ROTATE:
                    fd.type = "rotate";
                    fd.heading = Math.toRadians(action.rotateHeading);
                    break;
                case COMMAND:
                    fd.type = "command";
                    fd.stopping = action.commandStopping;
                    break;
            }
            out.add(fd);
        }
        return out;
    }

    // =========================================================================
    // Segment time helpers (same as original FeatherFlow)
    // =========================================================================

    private static double getSegmentStartTime(FeatherPath fp, int segIdx) {
        if (segIdx == 0)
            return 0.0;
        List<Double> splits = splitTimes(fp.actions);
        return segIdx - 1 < splits.size() ? splits.get(segIdx - 1) : 0.0;
    }

    private static double getSegmentEndTime(FeatherPath fp, int segIdx, double totalTime) {
        List<Double> splits = splitTimes(fp.actions);
        return segIdx < splits.size() ? splits.get(segIdx) : totalTime;
    }

    private static List<Double> splitTimes(List<FeatherActionDescriptor> actions) {
        List<Double> times = new ArrayList<>();
        for (FeatherActionDescriptor a : actions)
            if (a.type.equals("stop") || (a.type.equals("command") && a.stopping))
                times.add(a.time);
        times.sort(Double::compareTo);
        return times;
    }

    // =========================================================================
    // Type conversion
    // =========================================================================

    private static List<Trajectory.AnchorPoint> convertAnchors(List<AnchorPoint> raw, boolean flipped) {
        List<Trajectory.AnchorPoint> out = new ArrayList<>();
        for (AnchorPoint a : raw) {
            Trajectory.AnchorPoint ta = new Trajectory.AnchorPoint();
            ta.position = new Vector2(a.position.x, a.position.y);
            ta.handleInOffset = new Vector2(a.handleInOffset.x, a.handleInOffset.y);
            ta.handleOutOffset = new Vector2(a.handleOutOffset.x, a.handleOutOffset.y);
            ta.isCurved = a.isCurved;
            ta.handlesAligned = a.handlesAligned;
            ta.name = a.name;
            if (flipped) {
                ta.position.y = Trajectory.FIELD_HEIGHT_INCHES - ta.position.y;
                ta.handleInOffset.y = -ta.handleInOffset.y;
                ta.handleOutOffset.y = -ta.handleOutOffset.y;
            }
            out.add(ta);
        }
        return out;
    }

    private static List<Trajectory.ControlPoint> convertControlPoints(List<ControlPoint> raw) {
        List<Trajectory.ControlPoint> out = new ArrayList<>();
        for (ControlPoint cp : raw) {
            Trajectory.ControlPoint tc = new Trajectory.ControlPoint();
            tc.id = cp.id;
            tc.u = cp.u;
            tc.name = cp.name;
            tc.color = cp.color;
            for (ControlPointAttribute attr : cp.attributes) {
                Trajectory.ControlPointAttribute ta = new Trajectory.ControlPointAttribute();
                switch (attr.type) {
                    case "stop":
                        ta.type = Trajectory.ControlPointAttributeType.STOP;
                        ta.duration = attr.duration;
                        break;
                    case "rotate":
                        ta.type = Trajectory.ControlPointAttributeType.ROTATE;
                        ta.heading = attr.heading;
                        break;
                    case "command":
                        ta.type = Trajectory.ControlPointAttributeType.COMMAND;
                        ta.stopping = attr.stopping;
                        break;
                    case "loop":
                        ta.type = Trajectory.ControlPointAttributeType.LOOP;
                        ta.bounces = attr.bounces;
                        break;
                    case "motionLimits":
                        ta.type = Trajectory.ControlPointAttributeType.MOTION_LIMITS;
                        ta.velocity = attr.velocity;
                        ta.acceleration = attr.acceleration;
                        break;
                    default:
                        continue;
                }
                tc.attributes.add(ta);
            }
            out.add(tc);
        }
        return out;
    }

    private static Trajectory.MotionSettings convertMotionSettings(FeatherPathDefinition.MotionSettings raw) {
        // Prefer global settings if they were loaded; otherwise use the per-file raw
        // settings.
        FeatherPathDefinition.MotionSettings use = (GLOBAL_MOTION_SETTINGS != null) ? GLOBAL_MOTION_SETTINGS : raw;
        Trajectory.MotionSettings ms = new Trajectory.MotionSettings();
        ms.maxTranslationalVelocity = use.maxTranslationalVelocity;
        ms.maxRotationalVelocity = use.maxRotationalVelocity;
        ms.maxWheelSpeed = use.maxWheelSpeed;
        ms.maxAcceleration = use.maxAcceleration;
        ms.maxLateralAcceleration = use.maxLateralAcceleration;
        ms.swerveRadius = use.swerveRadius;
        return ms.sanitized();
    }

    // =========================================================================
    // Interpolation helpers
    // =========================================================================

    private static double interpTimeAtS(List<Trajectory.TrajPoint> pts, double targetS) {
        if (pts.isEmpty())
            return 0.0;
        targetS = Math.max(targetS, 0.0);
        if (pts.get(0).s >= targetS)
            return pts.get(0).time;
        if (pts.get(pts.size() - 1).s <= targetS)
            return pts.get(pts.size() - 1).time;
        int lo = 0, hi = pts.size() - 1;
        while (lo < hi - 1) {
            int mid = (lo + hi) >>> 1;
            if (pts.get(mid).s <= targetS)
                lo = mid;
            else
                hi = mid;
        }
        Trajectory.TrajPoint p1 = pts.get(lo), p2 = pts.get(hi);
        double frac = Math.abs(p2.s - p1.s) > 1e-9 ? (targetS - p1.s) / (p2.s - p1.s) : 0.5;
        return p1.time + frac * (p2.time - p1.time);
    }
}