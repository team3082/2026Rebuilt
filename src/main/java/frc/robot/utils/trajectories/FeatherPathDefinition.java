package frc.robot.utils.trajectories;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSetter;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@JsonIgnoreProperties(ignoreUnknown = true)
public class FeatherPathDefinition {

    // The .ff file uses "anchorPoints" not "anchors"
    @JsonProperty("anchorPoints")
    public List<AnchorPoint> anchors = new ArrayList<>();

    @JsonProperty("controlPoints")
    public List<ControlPoint> controlPoints = new ArrayList<>();

    // motionSettings is not in the .ff file — use defaults
    public MotionSettings motionSettings = new MotionSettings();

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class AnchorPoint {
        @JsonProperty("position")
        public Vec2 position = new Vec2();
        
        @JsonProperty("handleInOffset")
        public Vec2 handleInOffset = new Vec2();

        @JsonProperty("handleOutOffset")
        public Vec2 handleOutOffset = new Vec2();

        @JsonProperty("isCurved")
        public boolean isCurved = true;

        @JsonProperty("handlesAligned")
        public boolean handlesAligned = true;

        @JsonProperty("name")
        public String name = "";
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class Vec2 {
        @JsonProperty("x")
        public double x = 0;

        @JsonProperty("y")
        public double y = 0;
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ControlPoint {
        @JsonProperty("id")
        public long id = 0;

        @JsonProperty("u")
        public double u = 0;

        @JsonProperty("name")
        public String name = "";

        @JsonProperty("color")
        public String color = "";

        @JsonProperty("attributes")
        public List<ControlPointAttribute> attributes = new ArrayList<>();
    }

    @JsonIgnoreProperties(ignoreUnknown = true)
    public static class ControlPointAttribute {
        @JsonProperty("type")
        public String type = "";

        @JsonProperty("duration")
        public double duration = 0;

        @JsonProperty("heading")
        public double heading = 0;

        @JsonSetter("heading")
        public void setHeading(double heading) {
            this.heading = heading + 90;
            this.heading = -this.heading;
        }

        @JsonProperty("stopping")
        public boolean stopping = false;

        @JsonProperty("bounces")
        public int bounces = 0;

        @JsonProperty("targetLoopId")
        public Long targetLoopId = null;

        @JsonProperty("velocity")
        public double velocity = 0;

        @JsonProperty("acceleration")
        public double acceleration = 0;
    }

    // Not in the .ff file — hardcoded defaults matching the Rust defaults
    public static class MotionSettings {
        public double maxTranslationalVelocity = 170.0;
        public double maxRotationalVelocity = 5.0;
        public double maxWheelSpeed = 170.0;
        public double maxAcceleration = 170.0;
        public double maxLateralAcceleration = 170.0;
        public double swerveRadius = 14.0;
    }

    private static final ObjectMapper MAPPER = new ObjectMapper();

    public static FeatherPathDefinition fromFile(File file) throws IOException {
        return MAPPER.readValue(file, FeatherPathDefinition.class);
    }
}