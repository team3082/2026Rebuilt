package frc.robot.subsystems;

import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.utils.Vector2;

public class AutoTarget {
    
    public static void init() {

    }

    /**
     * Gets the desired shooter target for the robot's position on the field
     * @param pos current position
     * @return desired shooter target
     */
    public static ShooterTarget getTarget() {

        if (inAllianceZone()) {
            return ShooterTarget.HUB;
        }

        if (onRightSide()) {
            return ShooterTarget.PASS_RIGHT;
        } else {
            return ShooterTarget.PASS_LEFT;
        }
    }

    /**
     * To determine if we need to move hood down, it currently checks if our distance from the center of the trench is within a set value
     * We may want to set this to be a more defined rectangular range in the future
     * @return returns if we are close enough to trench to need to move the hood down
     */
    public static boolean nearTrench() {
        double SAFE_DIST = 60.0; // TODO determine safe distance away
        Vector2 trenchPos1 = new Vector2(143.5, 133.5);
        Vector2 trenchPos2 = new Vector2(trenchPos1.x, -trenchPos1.y);
        Vector2 trenchPos3 = new Vector2(-trenchPos1.x, trenchPos1.y);
        Vector2 trenchPos4 = new Vector2(-trenchPos1.x, -trenchPos1.y);
        return trenchPos1.sub(Odometry.getPosition()).mag() < SAFE_DIST
            || trenchPos2.sub(Odometry.getPosition()).mag() < SAFE_DIST
            || trenchPos3.sub(Odometry.getPosition()).mag() < SAFE_DIST
            || trenchPos4.sub(Odometry.getPosition()).mag() < SAFE_DIST;
    }

    private static boolean inAllianceZone() {
        return (Odometry.getPosition().x > 158.6); // 158.6 is depth of alliance zone from manual
    }

    private static boolean onRightSide() {
        return (Odometry.getPosition().y > 0);
    }
}
