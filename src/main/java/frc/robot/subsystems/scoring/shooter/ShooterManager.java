package frc.robot.subsystems.scoring.shooter;

import frc.robot.Constants;
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
                Shooter.shooterState = ShooterState.IDLE;
                break;

            case REVVING:
                Shooter.shooterState = ShooterState.REVVING;
                if (Shooter.atAngle() && Shooter.atRampedSpeed()) {
                    shooterState = ShooterState.SHOOTING;
                }
                setShooterAngleSpeed();

                break;

            case SHOOTING:
                Shooter.shooterState = ShooterState.SHOOTING;
                setShooterAngleSpeed();

                break;
        }

        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());
        Turret.setAngle(Math.atan2(deltaPos.y, deltaPos.x));   

    }

    public static void setShooterAngleSpeed() {
        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());

        double maxHeight = 0.0;

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
        }

        double angle = Math.atan2(2 * maxHeight * (1 + Math.sqrt(1 - target.pos.z / maxHeight)), deltaPos.mag());
        double speed = Math.sqrt(2 * 9.8 * maxHeight) / Math.sin(angle);

        Shooter.setAngle(angle);
        Shooter.setSpeed(speed);

    }

}