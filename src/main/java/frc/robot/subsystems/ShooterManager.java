package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.utils.Vector2;

public class ShooterManager {

    public static ShooterState shooterState;
    public static ShooterTarget target;
    
    public static void init() {
        Turret.init();
        Shooter.init();
        shooterState = ShooterState.IDLE;
        target = ShooterTarget.HUB;
    }

    public static void update() { 

        switch (shooterState) {
            case IDLE:
                Shooter.setTargetAngle(0);
                break;

            case REVVING:
                if (Shooter.atAngle() && Shooter.atRampedSpeed() && Turret.atAngle()) {
                    shooterState = ShooterState.SHOOTING;
                }
                setShooterAngleSpeed();

                break;

            case SHOOTING:
                setShooterAngleSpeed();

                break;
        }

        // The delta the ball needs to go 
        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());

        double targetTurretAngle = Turret.clampAngle(Math.atan2(deltaPos.y, deltaPos.x) + Pigeon.getRotationRad());

        if (targetTurretAngle == -10) {
            targetTurretAngle = Constants.Shooter.TURRET_MIN_ANGLE;
        }

        Turret.setAngle(targetTurretAngle);

        Turret.update();
        Shooter.update();

    }

    public static void startShooting() {
        shooterState = ShooterState.REVVING;
    }

    public static void stopShooting() {
        shooterState = ShooterState.IDLE;
    }

    private static void setShooterAngleSpeed() {
        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());

        double maxHeight;

        switch (target) {
            case HUB:
                maxHeight = 120.0 + 1.2 * deltaPos.mag();
                break;
        
            case PASS_LEFT:
                maxHeight = 36.0 + 1.05 * deltaPos.mag();
                break;

            case PASS_RIGHT:
                maxHeight = 36.0 + 1.05 * deltaPos.mag();
                break;
            default:
                maxHeight = 100.0;
                break;
        }

        double angle = Math.atan(2 * maxHeight * (1 + Math.sqrt(1 - target.pos.z / maxHeight)) / deltaPos.mag()) - Math.toRadians(25.0); // subtracts 25 since zero position is at 25 degree angle
        angle = Math.min(angle, Constants.Shooter.HOOD_MAX_ANGLE);
        double speed = Math.sqrt(2 * 9.8 * maxHeight) / Math.sin(angle);

        Shooter.setTargetSpeed(speed);
        Shooter.setTargetAngle(angle);

    }

}