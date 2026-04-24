package frc.robot.utils.trajectories;

import java.util.List;
import java.util.Map;
import com.fasterxml.jackson.annotation.JsonAlias;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

/**
 * Top-level container for a precompiled trajectory artifact.
 * This data structure matches the FeatherFlow compiled JSON format exactly,
 * enabling zero-computation trajectory loading in WPILib.
 */
public class CompiledTrajectory {
    public int formatVersion;
    public String sourceRoutineId;
    public String sourceRoutineName;
    public String generatedAtUtc;
    public String generatorVersion;
    public CoordinateFrameMetadata coordinateFrame;
    public Variants variants;

    /**
     * Metadata about the coordinate frame used in this trajectory.
     */
    public static class CoordinateFrameMetadata {
        public String units;           
        public String origin;         
        public String headingConvention;
    }

    /**
     * Container for normal and flipped trajectory variants.
     */
    public static class Variants {
        public CompiledVariant normal;
        public CompiledVariant flipped;
    }

    /**
     * A fully precomputed trajectory variant (either normal or flipped field coordinates).
     */
    public static class CompiledVariant {
        public double totalTime;
        public double totalDistance;
        public List<CompiledSegment> segments;
        public List<CompiledEvent> events;
        public VariantMetadata metadata;
    }

    /**
     * A contiguous segment of the path between two split points (e.g., between stops).
     */
    public static class CompiledSegment {
        public int segmentIndex;
        public double startT;        // normalized path parameter [0, 1]
        public double endT;          // normalized path parameter [0, 1]
        public double startTime;     // cumulative seconds from trajectory start
        public double endTime;       // cumulative seconds from trajectory start
        public List<ProfiledPoint> pathPoints;
    }

    /**
     * An event (stop, command, rotate, motion limit) with precomputed absolute time.
     */
    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class CompiledEvent {
        @JsonAlias({"eventType"})
        public String type;          // "stop", "command", "rotate", "motionLimits"
        public double t;             // normalized path parameter [0, 1]
        public double time;          // absolute cumulative time in trajectory
        public Map<String, Object> payload; // type-specific data
    }

    /**
     * Metadata about the variant (sample count, split points).
     */
    public static class VariantMetadata {
        public int sampleCount;
        public List<Double> splitTs;  // normalized t values where path was split
    }
}
