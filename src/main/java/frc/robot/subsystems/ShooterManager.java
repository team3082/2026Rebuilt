package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Tuning;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class ShooterManager {

    public enum ShooterManagerState {
        NORMAL,
        MANUAL_TOWER,
        MANUAL_TRENCH,
        MANUAL_PASS,
    }

    // default to safe values so Telemetry static init can't NPE before init() is called
    public static ShooterState shooterState = ShooterState.IDLE;
    private static ShooterTarget target = ShooterTarget.HUB;
    public static boolean inRange = true; // for LEDs, to track and display if we are in range to shoot at the Hub
    private static ShooterManagerState targetingState = ShooterManagerState.NORMAL;
    
    public static void init() {
        Shooter.init();
        shooterState = ShooterState.IDLE;
        target = ShooterTarget.HUB;
    }

    public static void update() { 

        setTarget(AutoTarget.getTarget());

        switch (shooterState) {
            case IDLE:
                Shooter.setTargetAngle(0);
                
                break;

            case ZEROING:
                break;

            case REVVING:
                if (Shooter.atAngle() && Shooter.atRampedSpeed() && (SwervePID.atRot() || OI.manualAim)) {
                    shooterState = ShooterState.SHOOTING;
                }
                
                setShooterAngleAndSpeed();

                break;

            case SHOOTING:
                // if (!Shooter.atAngle() || !Shooter.atRampedSpeed() || !SwervePID.atRot()) {
                //     shooterState = ShooterState.REVVING;
                // }
                setShooterAngleAndSpeed();

                break;
        }
        Shooter.update();

    }

    public static ShooterManagerState getTargetingState() {
        return targetingState;
    }

    public static void setNormalAiming() {
        targetingState = ShooterManagerState.NORMAL;
    }

    public static void setManualTower() {
        targetingState = ShooterManagerState.MANUAL_TOWER;
    }

    public static void setManualTrench() {
        targetingState = ShooterManagerState.MANUAL_TRENCH;
    }

    public static void setManualPass() {
        targetingState = ShooterManagerState.MANUAL_PASS;
    }

    public static void shoot() {
        if (shooterState != ShooterState.SHOOTING) {
            shooterState = ShooterState.REVVING;
        }
    }

    public static ShooterState getShooterState() {
        return shooterState != null ? shooterState : ShooterState.IDLE;
    }

    public static void setTarget(ShooterTarget newTarget) {
        target = newTarget;
    }

    public static ShooterTarget getTarget() {
        return target != null ? target : ShooterTarget.HUB;
    }

    public static void stopShooting() {
        shooterState = ShooterState.IDLE;
    }

    public static void zeroHood() {
        Shooter.zeroHood();
    }

    private static void setShooterAngleAndSpeed() {

        switch (targetingState) {
            case NORMAL:
                switch (target) {
                    case HUB:
                        aimAtHub();
                        break;
                        
                    default:
                        aimPass();
                        break;
                }
                break;
        
            case MANUAL_TRENCH:
                Shooter.setTargetSpeed(2510);           
                Shooter.setTargetAngle(0);
                break;

            case MANUAL_TOWER:
                Shooter.setTargetSpeed(2350);
                Shooter.setTargetAngle(0);
                break;

            case MANUAL_PASS:
                Shooter.setTargetSpeed(4400);
                Shooter.setTargetAngle(Math.toRadians(15.0));
        }
        
    }

    private static void aimPass() {
        Vector2 shooterPos = Odometry.getPosition();

        // gets distance between shooter and target
        Vector2 targetPosition = new Vector2(target.pos.x, SwervePosition.getPosition().y);
        double distance = targetPosition.sub(shooterPos).mag();

        // based on distance, uses shooter table to set flywheel speeds for different ranges of distances
        for (int i = Tuning.Shooter.SHOOTER_TABLE_PASSING.length - 1; i >= 0; i--) {
            if (Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getDist() < distance) {
                Shooter.setTargetSpeed(Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getSpeed());
                Shooter.setTargetAngle(Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getAngle() - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        Shooter.setTargetAngle(0);
        Shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
        inRange = false; // see variable creation before impulse deleting

    }

    private static void aimAtHub() {

        Vector2 finalPredictedRobotPosition = predictRobotPos();
        double finalPredictedDistance = target.pos.sub(finalPredictedRobotPosition).mag();

        if (finalPredictedDistance < Tuning.Shooter.SHOOTER_TABLE_HUB[0].getDist()) { // defaults to lowest value if too close, does this because if it interpolates to lower flywheel speed the shot won't go high enough
            Shooter.setTargetSpeed(Tuning.Shooter.SHOOTER_TABLE_HUB[0].getSpeed());
            Shooter.setTargetAngle(Tuning.Shooter.SHOOTER_TABLE_HUB[0].getAngle() - Constants.Shooter.HOOD_ANGLE_OFFSET);
            return;
        }

        for (int i = 0; i < Tuning.Shooter.SHOOTER_TABLE_HUB.length - 1; i++) {
            // finds which two values current distance is between and interpolates hood angle and flywheel speed between those values
            // if value is greater than maximum distance, continues linear approximation to that distance
            if ((Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist() < finalPredictedDistance && finalPredictedDistance < Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getDist()) || i == Tuning.Shooter.SHOOTER_TABLE_HUB.length - 2) {

                // amount that distance is from first to second distance
                double t = (finalPredictedDistance - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist()) / (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getDist() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist());
                
                // interpolates flywheel speed and angle
                double baseFlywheelSpeed = Tuning.Shooter.SHOOTER_TABLE_HUB[i].getSpeed();
                double speed = baseFlywheelSpeed + (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getSpeed() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getSpeed()) * t;
                double baseAngle = Tuning.Shooter.SHOOTER_TABLE_HUB[i].getAngle();
                double angle = baseAngle + (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getAngle() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getAngle()) * t;
                
                Shooter.setTargetSpeed(speed);
                Shooter.setTargetAngle(angle - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        // if not in range, sets angle to 0 and default flywheel speed
        Shooter.setTargetAngle(0);
        Shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
        inRange = false;
    }

    /**
     * predicts robot shooter position based on velocity for move and shoot
     * @return predicted robot position
     */
    public static Vector2 predictRobotPos() {
        Vector2 robotPosition = Odometry.getPosition().add(Constants.Shooter.SHOOTER_POS_OFFSET);
        double distance = target.pos.sub(robotPosition).mag();
    
        double lookAhead = distance * Tuning.Shooter.LOOK_AHEAD_TIME_K;

        Vector2 velocity = Odometry.getVelocity();

        Vector2 newPos = robotPosition.add(velocity.mul(lookAhead));
        double newDistance = target.pos.sub(newPos).mag();
        double newLookAhead = newDistance * Tuning.Shooter.LOOK_AHEAD_TIME_K;

        Vector2 finalPredictedRobotPosition = robotPosition.add(velocity.mul(newLookAhead));
        return finalPredictedRobotPosition;
    }
}

