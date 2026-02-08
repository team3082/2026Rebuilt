package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.utils.Vector2;

public class ShooterManager {

    private static ShooterState shooterState;
    private static ShooterTarget target;
    
    public static void init() {
        Turret.init();
        Shooter.init();
        shooterState = ShooterState.IDLE;
        target = ShooterTarget.HUB;
    }

    public static void update() { 

        aimTurret();

        switch (shooterState) {
            case IDLE:
                Shooter.setTargetAngle(0);
                break;

            case REVVING:
                if (Shooter.atAngle() && Shooter.atRampedSpeed() && Turret.atAngle()) {
                    shooterState = ShooterState.SHOOTING;
                }
                setShooterAngleAndSpeed();

                break;

            case SHOOTING:
                setShooterAngleAndSpeed();

                break;
        }

        Turret.update();
        Shooter.update();

    }

    public static void idle() {
        shooterState = ShooterState.IDLE;
    }

    public static void shoot() {
        shooterState = ShooterState.REVVING;
    }

    public static ShooterState getShooterState() {
        return shooterState;
    }

    public static void setTarget(ShooterTarget newTarget) {
        target = newTarget;
    }

    public static ShooterTarget getTarget() {
        return target;
    }

    public static void startShooting() {
        shooterState = ShooterState.REVVING;
    }

    public static void stopShooting() {
        shooterState = ShooterState.IDLE;
    }

    private static void aimTurret() { 
        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());

        double targetTurretAngle = Turret.clampAngle(Math.atan2(deltaPos.x, deltaPos.y) + Pigeon.getRotationRad());

        if (targetTurretAngle == -10) {
            targetTurretAngle = Constants.Shooter.TURRET_MIN_ANGLE; // TODO account for dead zone
        }

        Turret.setAngle(targetTurretAngle);
    }

    private static void setShooterAngleAndSpeed() {
        
        switch (target) {
            case HUB:
                aimAtHub();
                break;
        
            default: // TODO add aiming at other positions
                break;
        }

    }

    private static void aimAtHub() {
        double distance = new Vector2(target.pos.x, target.pos.y).sub(Odometry.getPosition()).mag();
        for (int i = 0; i < Constants.Shooter.SHOOTER_TABLE.length - 1; i++) {
            if (Constants.Shooter.SHOOTER_TABLE[i].getDist() < distance && distance < Constants.Shooter.SHOOTER_TABLE[i+1].getDist()) {

                // amount that distance is from first to second distance
                double t = (distance - Constants.Shooter.SHOOTER_TABLE[i].getDist()) / (Constants.Shooter.SHOOTER_TABLE[i+1].getDist() - Constants.Shooter.SHOOTER_TABLE[i].getDist());
                
                double baseFlywheelSpeed = Constants.Shooter.SHOOTER_TABLE[i].getSpeed();
                double speed = baseFlywheelSpeed + (Constants.Shooter.SHOOTER_TABLE[i+1].getSpeed() - Constants.Shooter.SHOOTER_TABLE[i].getSpeed()) * t;
                double baseAngle = Constants.Shooter.SHOOTER_TABLE[i].getAngle();
                double angle = baseAngle + (Constants.Shooter.SHOOTER_TABLE[i+1].getAngle() - Constants.Shooter.SHOOTER_TABLE[i].getAngle()) * t;
                
                Shooter.setTargetSpeed(speed);
                Shooter.setTargetAngle(angle - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        Shooter.setTargetAngle(0);
        Shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
    }

}