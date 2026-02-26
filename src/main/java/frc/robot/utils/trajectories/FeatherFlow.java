package frc.robot.utils.trajectories;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.FollowPath;
import frc.robot.auto.commands.RotateAndDriveTo;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.FeatherPath.FeatherActionDescriptor;

public class FeatherFlow {
    private static Map<String, FeatherPath> trajectories = new HashMap<>();

    public static void init() {
        File directory = new File(Filesystem.getDeployDirectory(), "FeatherFlow");
        File[] files = directory.listFiles((dir, name) -> name.toLowerCase().endsWith(".ff"));
        if (files == null) {
            System.err.println("FeatherFlow directory not found or empty");
            return;
        }

        new Thread(() -> {
            for (File file : files) {
                try {
                    String key = file.getName().replace(".ff", "");
                    trajectories.put(key, loadFeatherFile(file));
                    System.out.println("[FeatherFlow] " + file.getName() + " loaded successfully");
                } catch (Exception e) {
                    System.err.println("[FeatherFlow] Error loading " + file.getName());
                    e.printStackTrace();
                }
            }
            System.out.println("[FeatherFlow] Loaded " + trajectories.size() + " trajectories");
        }, "FeatherFlow Parser").start();
    }

    private static FeatherPath loadFeatherFile(File file) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(file);
        
        // Parse anchor points into cubic Bezier curves
        JsonNode anchorPoints = root.get("anchorPoints");
        if (anchorPoints == null || !anchorPoints.isArray() || anchorPoints.size() < 2) {
            throw new IOException("Invalid or missing anchorPoints in " + file.getName());
        }
        
        List<CubicBezierCurve> beziers = new ArrayList<>();
        
        // Create a bezier curve between each consecutive pair of anchor points
        for (int i = 0; i < anchorPoints.size() - 1; i++) {
            JsonNode current = anchorPoints.get(i);
            JsonNode next = anchorPoints.get(i + 1);

            Vector2 p0 = parsePosition(current.get("position"));
            Vector2 p1 = p0.add(parseOffset(current.get("handleOutOffset")));
            Vector2 p3 = parsePosition(next.get("position"));
            Vector2 p2 = p3.add(parseOffset(next.get("handleInOffset")));
            
            beziers.add(new CubicBezierCurve(p0, p1, p2, p3));
        }
        
        List<Vector2> allPoints = new ArrayList<>();
        List<Double> allCurvatures = new ArrayList<>();
        
        for (int i = 0; i < beziers.size(); i++) {
            CubicBezierCurve bezier = beziers.get(i);
            Vector2[] pts = bezier.getPoints();
            double[] curvs = bezier.getCurvatures();

            // Skip the first point of every curve except the first —
            // it is identical to the last point of the previous curve.
            int start = (i == 0) ? 0 : 1;
            for (int j = start; j < pts.length; j++) {
                allPoints.add(pts[j]);
                allCurvatures.add(curvs[j]);
            }
        }
        
        RobotPath fullPath = new RobotPath(allPoints, allCurvatures);
        
        JsonNode controlPoints = root.get("controlPoints");
        List<Double> splitValues = new ArrayList<>();
        List<FeatherActionDescriptor> actions = new ArrayList<>();
        
        if (controlPoints != null && controlPoints.isArray()) {
            for (JsonNode cp : controlPoints) {
                double u = cp.get("u").asDouble();
            
                int curveIndex = (int) u;
                double localT = u - curveIndex;
                curveIndex = Math.max(0, Math.min(curveIndex, beziers.size() - 1));
                double globalT = (curveIndex + localT) / beziers.size();
                globalT = Math.max(0.0, Math.min(1.0, globalT));
                
                // Parse attributes for this control point
                JsonNode attributes = cp.get("attributes");
                if (attributes != null && attributes.isArray()) {
                    for (JsonNode attr : attributes) {
                        String type = attr.get("type").asText();
                        
                        FeatherActionDescriptor descriptor = new FeatherActionDescriptor();
                        descriptor.t = globalT;
                        descriptor.type = type;
                        
                        switch (type) {
                            case "stop":
                                descriptor.duration = attr.has("duration") ? 
                                    attr.get("duration").asDouble() : 0.0;
                                splitValues.add(globalT);
                                break;
                            case "rotate":
                                descriptor.heading = attr.has("heading") ? 
                                    attr.get("heading").asDouble() : 0.0;
                                break;
                            case "command":
                                descriptor.stopping = attr.has("stopping") ? 
                                    attr.get("stopping").asBoolean() : false;
                                if (descriptor.stopping) {
                                    splitValues.add(globalT);
                                }
                                break;
                        }
                        
                        actions.add(descriptor);
                    }
                }
            }
        }
        
        // Split path at stop points
        List<RobotPath> paths;
        if (!splitValues.isEmpty()) {
            paths = fullPath.split(splitValues);
        } else {
            paths = List.of(fullPath);
        }

        // Build a sorted list of (globalT, heading) pairs from "rotate" descriptors
        List<double[]> rotateKeyframes = new ArrayList<>();
        for (FeatherActionDescriptor action : actions) {
            if (action.type.equals("rotate")) {
                rotateKeyframes.add(new double[]{action.t, Math.toRadians(action.heading+90)});
            }
        }
        rotateKeyframes.sort((a, b) -> Double.compare(a[0], b[0]));

        // Compute full path arc-length distances once
        List<Vector2> allPts = fullPath.getPoints();
        double[] fullDist = new double[allPts.size()];
        fullDist[0] = 0.0;
        for (int i = 1; i < allPts.size(); i++) {
            fullDist[i] = fullDist[i - 1] + allPts.get(i - 1).dist(allPts.get(i));
        }
        double totalPathDist = fullDist[allPts.size() - 1];

        // Convert rotate keyframes from globalT → arc-length distance
        // {distanceAlongFullPath, heading}
        List<double[]> rotateByDist = new ArrayList<>();
        for (double[] kf : rotateKeyframes) {
            rotateByDist.add(new double[]{kf[0] * totalPathDist, kf[1]});
        }
        // rotateByDist is already sorted since rotateKeyframes was sorted by t

        // Starting heading = first keyframe heading (path-planning convention).
        // Ending heading   = last keyframe heading.
        // If no keyframes, both default to 0.
        double startRotation = rotateByDist.isEmpty() ? 0.0 : rotateByDist.get(0)[1];
        double endRotation   = rotateByDist.isEmpty() ? 0.0 : rotateByDist.get(rotateByDist.size() - 1)[1];

        List<ProfiledPath> profiledPaths = new ArrayList<>();
        double segmentStartT = 0.0;
        double segmentDistOffset = 0.0;

        for (int segIdx = 0; segIdx < paths.size(); segIdx++) {
            RobotPath segPath = paths.get(segIdx);
            List<Vector2> segPts = segPath.getPoints();
            int pointCount = segPts.size();

            double segEndT = 1.0;
            if (!splitValues.isEmpty() && segIdx < splitValues.size()) {
                segEndT = splitValues.get(segIdx);
            }

            // Compute cumulative arc-length within this segment
            double[] segDist = new double[pointCount];
            segDist[0] = 0.0;
            for (int i = 1; i < pointCount; i++) {
                segDist[i] = segDist[i - 1] + segPts.get(i - 1).dist(segPts.get(i));
            }

            double[] targetHeadings = new double[pointCount];

            for (int i = 0; i < pointCount; i++) {
                // True arc-length distance of this point along the full path
                double d = segmentDistOffset + segDist[i];

                if (rotateByDist.isEmpty()) {
                    targetHeadings[i] = 0.0;
                    continue;
                }

                // Before first keyframe → hold first keyframe heading
                if (d <= rotateByDist.get(0)[0]) {
                    targetHeadings[i] = rotateByDist.get(0)[1];
                    continue;
                }

                // After last keyframe → hold last keyframe heading
                if (d >= rotateByDist.get(rotateByDist.size() - 1)[0]) {
                    targetHeadings[i] = rotateByDist.get(rotateByDist.size() - 1)[1];
                    continue;
                }

                // Between two keyframes → find them and interpolate shortest-path
                double prevD = rotateByDist.get(0)[0];
                double prevH = rotateByDist.get(0)[1];
                double nextD = rotateByDist.get(rotateByDist.size() - 1)[0];
                double nextH = rotateByDist.get(rotateByDist.size() - 1)[1];

                for (int k = 0; k < rotateByDist.size() - 1; k++) {
                    if (rotateByDist.get(k)[0] <= d && rotateByDist.get(k + 1)[0] > d) {
                        prevD = rotateByDist.get(k)[0];
                        prevH = rotateByDist.get(k)[1];
                        nextD = rotateByDist.get(k + 1)[0];
                        nextH = rotateByDist.get(k + 1)[1];
                        break;
                    }
                }

                double span  = nextD - prevD;
                double alpha = (span > 1e-9) ? (d - prevD) / span : 0.0;
                double delta = nextH - prevH;
                while (delta >  Math.PI) delta -= 2 * Math.PI;
                while (delta < -Math.PI) delta += 2 * Math.PI;
                targetHeadings[i] = prevH + alpha * delta;
            }

            profiledPaths.add(ProfiledPath.generateSimplifiedProfile(
                segPath, 200, 5, 200, 200, targetHeadings
            ));

            segmentDistOffset += segDist[pointCount - 1];
            segmentStartT = segEndT;
        }
        
        return new FeatherPath(profiledPaths, actions);
    }
    
    private static Vector2 parsePosition(JsonNode posNode) {
        if (posNode == null) {
            throw new IllegalArgumentException("Position node is null");
        }

        return new Vector2(
            posNode.get("x").asDouble() - (Constants.FIELD_WIDTH/2),
            -((Constants.FIELD_HEIGHT/2) -posNode.get("y").asDouble())
        );

    }
    
    private static Vector2 parseOffset(JsonNode offsetNode) {
        if (offsetNode == null) {
            return new Vector2(0, 0); 
        }
        return new Vector2(
            offsetNode.get("x").asDouble(),
            offsetNode.get("y").asDouble()
        );
    }

    /**
     * Gets a loaded FeatherPath by name.
     * @param pathName Name of the path (without .ff extension)
     * @return The FeatherPath object
     * @throws IllegalArgumentException if path doesn't exist
     */
    public static FeatherPath getPath(String pathName) {
        if (!trajectories.containsKey(pathName)) {
            throw new IllegalArgumentException("Path '" + pathName + "' does not exist! " +
                "Available paths: " + trajectories.keySet());
        }
        return trajectories.get(pathName);
    }
    
    /**
     * Builds a sequential command group from a FeatherPath and provided commands.
     * @param pathName Name of the path to follow
     * @param commands Commands to be associated with command-type actions in the path (in order)
     * @return SequentialCommandGroup containing the path following commands
     */
    public static SequentialCommandGroup buildFeatherAuto(String pathName, Command... commands) {
        System.out.println("[FeatherFlow] Building auto for path: " + pathName);
        FeatherPath featherPath = getPath(pathName);
        
        SequentialCommandGroup group = new SequentialCommandGroup();

        group.addCommands(new RotateAndDriveTo(featherPath.paths.get(0).getStartHeading(), featherPath.paths.get(0).getStartPoint()));
        
        int commandIndex = 0;
        
        for (int pathIndex = 0; pathIndex < featherPath.paths.size(); pathIndex++) {
            ProfiledPath currentPath = featherPath.paths.get(pathIndex);
            
            double segmentStartT = pathIndex == 0 ? 0.0 : getPathSegmentStartT(featherPath, pathIndex);
            double segmentEndT = getPathSegmentEndT(featherPath, pathIndex);
        
            List<FeatherEvent> eventsForSegment = new ArrayList<>();
            
            for (FeatherActionDescriptor action : featherPath.actions) {
                if (action.t >= segmentStartT && action.t < segmentEndT) {
                    double normalizedT = (action.t - segmentStartT) / (segmentEndT - segmentStartT);
                    
                    if (action.type.equals("command") && !action.stopping) {
                        if (commandIndex < commands.length) {
                            eventsForSegment.add(new FeatherEvent(normalizedT, commands[commandIndex]));
                            commandIndex++;
                        }
                    } else if (action.type.equals("rotate")) {
                        //TODO
                    }
                }
            }
            
            FeatherEvent[] events = eventsForSegment.toArray(new FeatherEvent[0]);
            group.addCommands(new FollowPath(currentPath, events));
            
            for (FeatherActionDescriptor action : featherPath.actions) {
                if (Math.abs(action.t - segmentEndT) < 0.001) {
                    if (action.type.equals("stop")) {
                        group.addCommands(new WaitCommand(action.duration));
                    } else if (action.type.equals("command") && action.stopping) {
                        if (commandIndex < commands.length) {
                            group.addCommands(commands[commandIndex]);
                            commandIndex++;
                        }
                    }
                }
            }
        }
        
        return group;
    }

    private static double getPathSegmentStartT(FeatherPath featherPath, int segmentIndex) {
        if (segmentIndex == 0) return 0.0;
        
        List<Double> splitPoints = new ArrayList<>();
        for (FeatherActionDescriptor action : featherPath.actions) {
            if (action.type.equals("stop") || (action.type.equals("command") && action.stopping)) {
                splitPoints.add(action.t);
            }
        }
        splitPoints.sort(Double::compareTo);
        
        if (segmentIndex - 1 < splitPoints.size()) {
            return splitPoints.get(segmentIndex - 1);
        }
        return 0.0;
    }


    private static double getPathSegmentEndT(FeatherPath featherPath, int segmentIndex) {
        List<Double> splitPoints = new ArrayList<>();
        for (FeatherActionDescriptor action : featherPath.actions) {
            if (action.type.equals("stop") || (action.type.equals("command") && action.stopping)) {
                splitPoints.add(action.t);
            }
        }
        splitPoints.sort(Double::compareTo);
        
        if (segmentIndex < splitPoints.size()) {
            return splitPoints.get(segmentIndex);
        }
        return 1.0; 
    }
}