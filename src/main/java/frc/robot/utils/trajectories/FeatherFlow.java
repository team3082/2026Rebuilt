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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.commands.FollowPath;
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
            
            // Start point (P0)
            Vector2 p0 = parsePosition(current.get("position"));
            
            // First control point (P1) = start position + outgoing handle
            Vector2 p1 = p0.add(parseOffset(current.get("handleOutOffset")));
            
            // End point (P3)
            Vector2 p3 = parsePosition(next.get("position"));
            
            // Second control point (P2) = end position + incoming handle
            Vector2 p2 = p3.add(parseOffset(next.get("handleInOffset")));
            
            beziers.add(new CubicBezierCurve(p0, p1, p2, p3));
        }
        
        List<Vector2> allPoints = new ArrayList<>();
        
        for (CubicBezierCurve bezier : beziers) {
           allPoints.addAll(List.of(bezier.getPointsOnCurve()));
        }
        
        RobotPath fullPath = new RobotPath(allPoints);
        
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
        
        return new FeatherPath(paths, actions);
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
        FeatherPath featherPath = getPath(pathName);
        
        SequentialCommandGroup group = new SequentialCommandGroup();
        
        int commandIndex = 0;
        
        for (int pathIndex = 0; pathIndex < featherPath.paths.size(); pathIndex++) {
            RobotPath currentPath = featherPath.paths.get(pathIndex);
            
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