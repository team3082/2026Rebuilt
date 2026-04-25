package frc.robot.utils.trajectories;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

 
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
 
import frc.robot.auto.commands.FollowPath;
import frc.robot.swerve.SwervePosition;
 
import frc.robot.utils.trajectories.FeatherPath.FeatherActionDescriptor;

public class FeatherFlow {
    private static Map<String, FeatherPath[]> trajectories = new HashMap<>();
    
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
                    FeatherPath[] array = new FeatherPath[2];
                    array[0] =  loadFeatherFile(file, false);
                    array[1] = loadFeatherFile(file, true);
                    trajectories.put(key, array);
                    System.out.println("[FeatherFlow] " + file.getName() + " loaded successfully");
                } catch (Exception e) {
                    System.err.println("[FeatherFlow] Error loading " + file.getName());
                    e.printStackTrace();
                }
            }
            System.out.println("[FeatherFlow] Loaded " + trajectories.size() + " trajectories");
        }, "FeatherFlow Parser").start();
    }

    private static FeatherPath loadFeatherFile(File file, boolean flipped) {
        // Load compiled artifact-only. Legacy .ff recompute path removed.
        String baseName = file.getName().replace(".ff", "");
        File compiledFile = new File(file.getParent(), baseName + ".fftraj.json");
        if (!compiledFile.exists()) {
            throw new IllegalStateException("Compiled trajectory artifact not found for " + file.getName() + ". Expected " + baseName + ".fftraj.json in same directory.");
        }

        FeatherPath compiledPath = loadCompiledFeatherFile(compiledFile, flipped);
        if (compiledPath == null) {
            throw new IllegalStateException("Failed to load compiled trajectory artifact: " + compiledFile.getAbsolutePath());
        }
        System.out.println("[FeatherFlow] Loaded compiled artifact: " + baseName + ".fftraj.json");
        return compiledPath;
    }



    /**
     * Legacy .ff recompute path removed. Only compiled artifacts (.fftraj.json)
     * are supported by this loader. If you need the legacy loader, it's
     * preserved in the repository history.
     */

    /**
     * Loads a FeatherPath from a precompiled trajectory artifact (.fftraj.json).
     * This is the preferred loading path - no trajectory math computation needed.
     * 
     * @param file The compiled trajectory JSON file
     * @param flipped Whether to load the flipped variant
     * @return The loaded FeatherPath, or null if loading fails
     */
    private static FeatherPath loadCompiledFeatherFile(File file, boolean flipped) {
        try {
            CompiledTrajectory compiled = CompiledTrajectoryDeserializer.deserializeFile(file);
            
            if (compiled.formatVersion != 1) {
                System.err.println("[FeatherFlow] Unsupported formatVersion: " + compiled.formatVersion);
                return null;
            }
            
            CompiledTrajectory.CompiledVariant variant = flipped 
                ? compiled.variants.flipped 
                : compiled.variants.normal;
            
            if (variant == null) {
                System.err.println("[FeatherFlow] Variant not found in compiled artifact");
                return null;
            }
            
            // Convert compiled segments to ProfiledPath objects
            List<ProfiledPath> profiledPaths = new ArrayList<>();
            for (CompiledTrajectory.CompiledSegment segment : variant.segments) {
                // Segments already contain fully profiled points
                ArrayList<ProfiledPoint> points = new ArrayList<>(segment.pathPoints);
                profiledPaths.add(new ProfiledPath(points));
            }
            
            // Convert compiled events to FeatherActionDescriptor
            List<FeatherActionDescriptor> actions = new ArrayList<>();
            for (CompiledTrajectory.CompiledEvent event : variant.events) {
                FeatherActionDescriptor descriptor = new FeatherActionDescriptor();
                descriptor.t = event.t;
                descriptor.type = event.type;
                descriptor.time = event.time; // Absolute time in trajectory
                
                Map<String, Object> payload = event.payload;
                switch (event.type) {
                    case "stop":
                        descriptor.duration = ((Number) payload.get("duration")).doubleValue();
                        break;
                    case "rotate":
                        descriptor.heading = ((Number) payload.get("heading")).doubleValue();
                        break;
                    case "command":
                        descriptor.stopping = (boolean) payload.get("stopping");
                        break;
                    case "motionLimits":
                        descriptor.maxVelocity = ((Number) payload.get("maxVelocity")).doubleValue();
                        descriptor.maxAcceleration = ((Number) payload.get("maxAcceleration")).doubleValue();
                        break;
                }
                actions.add(descriptor);
            }
            
            return new FeatherPath(profiledPaths, actions);
        } catch (Exception e) {
            System.err.println("[FeatherFlow] Failed to load compiled artifact: " + e.getMessage());
            return null;
        }
    }


    /**
     * Gets a loaded FeatherPath by name.
     * @param pathName Name of the path (without .ff extension)
     * @return The FeatherPath object
     * @throws IllegalArgumentException if path doesn't exist
     */
    public static FeatherPath getPath(String pathName, boolean flipped) {
        if (!trajectories.containsKey(pathName)) {
            throw new IllegalArgumentException("Path '" + pathName + "' does not exist! " +
                "Available paths: " + trajectories.keySet());
        }
        return trajectories.get(pathName)[flipped ? 1 : 0];
    }

    public static SequentialCommandGroup buildFeatherAuto(String pathName, Command... commands) {
        return buildFeatherAuto(pathName, false, true, commands);
    }
    
    /**
     * Builds a sequential command group from a FeatherPath and provided commands.
     * 
     * For compiled trajectories, uses precomputed absolute times from the artifact.
     * For legacy trajectories, falls back to normalized t-based timing.
     * 
     * @param pathName Name of the path to follow
     * @param commands Commands to be associated with command-type actions in the path (in order)
     * @return SequentialCommandGroup containing the path following commands
     */
    public static SequentialCommandGroup buildFeatherAuto(String pathName, boolean flipped, boolean resetOdo,  Command... commands) {
        System.out.println("[FeatherFlow] Building auto for path: " + pathName);
        FeatherPath featherPath = getPath(pathName, flipped);
        
        SequentialCommandGroup group = new SequentialCommandGroup();

        if (resetOdo) {
            group.addCommands(new InstantCommand(()->{
                SwervePosition.setPosition(featherPath.paths.get(0).getStartPoint());
            }));
        }

        int commandIndex = 0;
        
        // Determine if we have absolute timing (from compiled artifact) or legacy timing
        boolean hasAbsoluteTiming = hasAbsoluteTimingData(featherPath);
        
        for (int pathIndex = 0; pathIndex < featherPath.paths.size(); pathIndex++) {
            ProfiledPath currentPath = featherPath.paths.get(pathIndex);
            
            double segmentStartT = pathIndex == 0 ? 0.0 : getPathSegmentStartT(featherPath, pathIndex);
            double segmentEndT = getPathSegmentEndT(featherPath, pathIndex);
            
            double segmentStartTime = pathIndex == 0 ? 0.0 : getPathSegmentStartTime(featherPath, pathIndex);
            double segmentEndTime = getPathSegmentEndTime(featherPath, pathIndex);
        
            List<FeatherEvent> eventsForSegment = new ArrayList<>();
            
            for (FeatherActionDescriptor action : featherPath.actions) {
                boolean inSegment = hasAbsoluteTiming
                    ? (action.time >= segmentStartTime && action.time < segmentEndTime)
                    : (action.t >= segmentStartT && action.t < segmentEndT);
                
                if (inSegment) {
                    double normalizedT;
                    if (hasAbsoluteTiming) {
                        // Use precomputed absolute time
                        normalizedT = (segmentEndTime - segmentStartTime) > 1e-9
                            ? (action.time - segmentStartTime) / (segmentEndTime - segmentStartTime)
                            : 0.5;
                    } else {
                        // Fall back to normalized t
                        normalizedT = (segmentEndT - segmentStartT) > 1e-9
                            ? (action.t - segmentStartT) / (segmentEndT - segmentStartT)
                            : 0.5;
                    }
                    
                    if (action.type.equals("command") && !action.stopping) {
                        if (commandIndex < commands.length) {
                            eventsForSegment.add(new FeatherEvent(normalizedT, commands[commandIndex]));
                            commandIndex++;
                        }
                    } else if (action.type.equals("rotate")) {
                        //TODO: Handle rotate actions
                    }
                }
            }
            
            FeatherEvent[] events = eventsForSegment.toArray(new FeatherEvent[0]);
            group.addCommands(new InstantCommand(() -> {
                System.out.println("Following Path Segment");
            }));
            group.addCommands(new FollowPath(currentPath, events));
            
            for (FeatherActionDescriptor action : featherPath.actions) {
                boolean atSegmentEnd = hasAbsoluteTiming
                    ? (Math.abs(action.time - segmentEndTime) < 0.001)
                    : (Math.abs(action.t - segmentEndT) < 0.001);
                
                if (atSegmentEnd) {
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

    /**
     * Checks if actions have precomputed absolute timing (from compiled artifacts).
     * Legacy trajectories have time field = 0.
     */
    private static boolean hasAbsoluteTimingData(FeatherPath featherPath) {
        if (featherPath.actions.isEmpty()) {
            return false;
        }
        
        // If any action has non-zero time, we have absolute timing
        for (FeatherActionDescriptor action : featherPath.actions) {
            if (action.time > 1e-9) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the start time for a path segment using precomputed absolute times.
     */
    private static double getPathSegmentStartTime(FeatherPath featherPath, int segmentIndex) {
        if (segmentIndex == 0) return 0.0;
        
        List<Double> splitTimes = new ArrayList<>();
        for (FeatherActionDescriptor action : featherPath.actions) {
            if (action.type.equals("stop") || (action.type.equals("command") && action.stopping)) {
                splitTimes.add(action.time);
            }
        }
        splitTimes.sort(Double::compareTo);
        
        if (segmentIndex - 1 < splitTimes.size()) {
            return splitTimes.get(segmentIndex - 1);
        }
        return 0.0;
    }

    /**
     * Gets the end time for a path segment using precomputed absolute times.
     */
    private static double getPathSegmentEndTime(FeatherPath featherPath, int segmentIndex) {
        List<Double> splitTimes = new ArrayList<>();
        for (FeatherActionDescriptor action : featherPath.actions) {
            if (action.type.equals("stop") || (action.type.equals("command") && action.stopping)) {
                splitTimes.add(action.time);
            }
        }
        splitTimes.sort(Double::compareTo);
        
        if (segmentIndex < splitTimes.size()) {
            return splitTimes.get(segmentIndex);
        }
        
        // Return total trajectory time
        double maxTime = 0.0;
        for (FeatherActionDescriptor action : featherPath.actions) {
            maxTime = Math.max(maxTime, action.time);
        }
        return maxTime > 0 ? maxTime : Double.MAX_VALUE;
    }
}