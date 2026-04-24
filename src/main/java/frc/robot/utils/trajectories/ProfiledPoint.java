package frc.robot.utils.trajectories;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JsonNode;

import frc.robot.Constants;
import frc.robot.utils.Vector2;

@JsonIgnoreProperties(ignoreUnknown = true)
public class ProfiledPoint {
    private Vector2 position;
    private Vector2 velocity;
    private double curvature;
    private double acceleration;
    private double time;
    private double distance;
    private double heading;             // radians, tangent angle of the path
    private double rotationalVelocity;  // rad/s

    // Raw JSON x/y (kept for fidelity), but setters now convert to internal coords
    @JsonProperty("x")
    private double x;

    @JsonProperty("y")
    private double y;

    public ProfiledPoint() {
        this.position = new Vector2(0, 0);
        this.velocity = new Vector2(0, 0);
        this.curvature = 0;
        this.acceleration = 0;
        this.time = 0;
        this.distance = 0;
        this.heading = 0;
        this.rotationalVelocity = 0;
        this.x = 0;
        this.y = 0;
    }

    // JSON-mapped setters keep both the primitive fields and the Vector2 position in sync.
    // These setters convert from field coordinates (JSON) to internal robot-centered coords.
    @JsonProperty("x")
    public void setX(double x) {
        this.x = x;
        // convert from field coordinates to robot coordinates (same logic as parsePosition)
        double rx = x - (Constants.FIELD_WIDTH / 2.0);
        if (this.position == null) this.position = new Vector2(0, 0);
        this.position.x = rx;
    }

    @JsonProperty("y")
    public void setY(double y) {
        this.y = y;
        // convert from field coordinates to robot coordinates
        double ry = -((Constants.FIELD_HEIGHT / 2.0) - y);
        if (this.position == null) this.position = new Vector2(0, 0);
        this.position.y = ry;
    }

    // Getters return the Vector2 coordinates when available, otherwise the primitive fields
    public double getX() {
        return this.position != null ? this.position.x : this.x;
    }

    public double getY() {
        return this.position != null ? this.position.y : this.y;
    }

    public ProfiledPoint(Vector2 position, Vector2 velocity, double curvature, double acceleration, double time, double distance) {
        this.position = position;
        this.velocity = velocity;
        this.curvature = curvature;
        this.acceleration = acceleration;
        this.time = time;
        this.distance = distance;
        this.heading = 0;
        this.rotationalVelocity = 0;
        if (position != null) {
            this.x = position.x;
            this.y = position.y;
        } else {
            this.x = 0;
            this.y = 0;
        }
    }

    public Vector2 getPosition() {
        return position;
    }

    // Accept "position" JSON object and parse it into internal Vector2 coords
    @JsonProperty("position")
    public void setPosition(JsonNode posNode) {
        if (posNode == null || posNode.isNull()) return;
        Vector2 p = parsePosition(posNode);
        this.position = p;
        // keep raw x/y values for fidelity
        this.x = posNode.path("x").asDouble(this.x);
        this.y = posNode.path("y").asDouble(this.y);
    }

    // Existing programmatic setter (overloaded) kept for internal usage
    public void setPosition(Vector2 position) {
        this.position = position;
        if (position != null) {
            this.x = position.x;
            this.y = position.y;
        }
    }

    public Vector2 getVelocity() {
        return velocity;
    }

    public void setVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getRotationalVelocity() {
        return rotationalVelocity;
    }

    public void setRotationalVelocity(double rotationalVelocity) {
        this.rotationalVelocity = rotationalVelocity;
    }

    private static Vector2 parsePosition(JsonNode posNode) {
        if (posNode == null || posNode.isNull()) {
            throw new IllegalArgumentException("Position node is null");
        }

        double rawX = posNode.path("x").asDouble(0.0);
        double rawY = posNode.path("y").asDouble(0.0);

        // Convert field coordinates (origin top-left) into internal robot-centered coordinates:
        // x: move origin to center (subtract half width)
        // y: invert and shift origin to center
        double rx = rawX - (Constants.FIELD_WIDTH / 2.0);
        double ry = -((Constants.FIELD_HEIGHT / 2.0) - rawY);

        return new Vector2(rx, ry);
    }

    @JsonProperty("heading")
    public void setHeadingFromJson(double headingValue) {
        double rad = Math.abs(headingValue) > 2.0 * Math.PI ? Math.toRadians(headingValue) : headingValue;
        this.heading = -rad + (3 * Math.PI / 2);
    }
}