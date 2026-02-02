package frc.robot.utils.trajectories;

import java.util.List;

/**
 * Simple holder for a parsed feather trajectory file.
 * Contains the split RobotPaths and a list of action descriptors parsed from the .ff file.
 */
public class FeatherPath {
    public final List<RobotPath> paths;
    public final List<FeatherActionDescriptor> actions;

    public FeatherPath(List<RobotPath> paths, List<FeatherActionDescriptor> actions) {
        this.paths = paths;
        this.actions = actions;
    }

    public static class FeatherActionDescriptor {
        public double t;  
        public String type;       
        public double duration;    
        public double heading;     
        public boolean stopping;   
    }
}