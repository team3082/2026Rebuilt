package frc.robot.utils.trajectories;

import java.util.List;
import frc.robot.utils.Vector2;

public class FeatherPath {

    public final List<ProfiledPath> paths; // kept as "paths" to match original field name
    public final List<FeatherActionDescriptor> actions;
    public final double totalTime;

    public FeatherPath(List<ProfiledPath> paths, List<FeatherActionDescriptor> actions, double totalTime) {
        this.paths = paths;
        this.actions = actions;
        this.totalTime = totalTime;
    }

    public Vector2 getStartPosition() {
        if (paths.isEmpty())
            return new Vector2(0, 0);
        return paths.get(0).getStartPoint();
    }

    public double getStartHeading() {
        if (paths.isEmpty())
            return 0;
        return paths.get(0).getStartHeading();
    }

    public static class FeatherActionDescriptor {
        public double t;
        public String type;
        public double time;
        public double duration;
        public double heading;
        public boolean stopping;
        public double maxVelocity;
        public double maxAcceleration;
    }
}
