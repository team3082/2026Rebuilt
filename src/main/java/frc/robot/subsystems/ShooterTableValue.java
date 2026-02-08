package frc.robot.subsystems;

public class ShooterTableValue {
    private final double distance;
    private final double angle;
    private final double speed;

    /**
     * This class contains values for angle and speed at different distances from the hub to interpolate between
     * @param distance Distance of center of robot from center of hub in inches
     * @param angle Angle of hood in radians
     * @param speed Speed of flywheel in RPM
     */
    public ShooterTableValue(double distance, double angle, double speed) {
        this.distance = distance;
        this.angle = angle;
        this.speed = speed;
    }

    /**
     * Gets distance from center of hub in inches
     * @return
     */
    public double getDist() {
        return distance;
    }

    /**
     * Gets angle of hood at the distance
     * @return
     */
    public double getAngle() {
        return angle;
    }

    /**
     * Gets flywheel speed at the distance
     * @return
     */
    public double getSpeed() {
        return speed;
    }
}
