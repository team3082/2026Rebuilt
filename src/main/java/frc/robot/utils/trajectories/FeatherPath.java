package frc.robot.utils.trajectories;

import java.util.List;

/**
 * Simple holder for a parsed feather trajectory file.
 * Contains the split RobotPaths and a list of action descriptors parsed from the .ff file.
 */
public class FeatherPath {
    public final List<ProfiledPath> paths;
    public final List<FeatherActionDescriptor> actions;

    public FeatherPath(List<ProfiledPath> paths, List<FeatherActionDescriptor> actions) {
        this.paths = paths;
        this.actions = actions;
    }

    public static class FeatherActionDescriptor {
        public double t;                    // Normalized path parameter [0, 1]
        public String type;                 // "stop", "command", "rotate", "motionLimits"
        public double time;                 // Absolute cumulative time in trajectory
        public double duration;             // For "stop" type
        public double heading;              // For "rotate" type
        public boolean stopping;            // For "command" type
        public double maxVelocity;          // For "motionLimits" type
        public double maxAcceleration;      // For "motionLimits" type
    }
}