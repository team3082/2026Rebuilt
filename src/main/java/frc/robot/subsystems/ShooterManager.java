package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.Vector2;

public class ShooterManager {

    private static ShooterState shooterState;
    private static ShooterTarget target;
    private static Shooter shooter;
    private static Turret turret;
    
    public static void init() {
        turret = new Turret();
        shooter = new Shooter();
        shooterState = ShooterState.IDLE;
        target = ShooterTarget.HUB;
    }

    public static void update() { 

        aimTurret();

        switch (shooterState) {
            case IDLE:
                shooter.setTargetAngle(0);
                break;

            case REVVING:
                if (shooter.atAngle() && shooter.atRampedSpeed() && turret.atAngle()) {
                    shooterState = ShooterState.SHOOTING;
                }
                setShooterAngleAndSpeed();

                break;

            case SHOOTING:
                setShooterAngleAndSpeed();

                break;
        }

        turret.update();
        shooter.update();

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

    public static Shooter getShooter() {
        return shooter;
    }

    public static Turret getTurret() {
        return turret;
    }

    public static void startShooting() {
        shooterState = ShooterState.REVVING;
    }

    public static void stopShooting() {
        shooterState = ShooterState.IDLE;
    }

    private static void aimTurret() { 
        Vector2 deltaPos = (new Vector2(target.pos.x, target.pos.y)).sub(Odometry.getPosition());

        double targetTurretAngle = turret.clampAngle(Math.atan2(deltaPos.x, deltaPos.y) + Pigeon.getRotationRad());

        if (targetTurretAngle == -10) {
            targetTurretAngle = Constants.Shooter.TURRET_MIN_ANGLE;
            if (SwerveManager.rotationSpeed == 0) {
                SwerveManager.rotateAndDrive(0.4, SwerveManager.movement);
            }
        }

        turret.setAngle(targetTurretAngle);
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

        Vector2 turretPos = Odometry.getPosition().add(Constants.Shooter.TURRET_POS_OFFSET.rotate(Pigeon.getRotationRad()));

        double distance = new Vector2(target.pos.x, target.pos.y).sub(turretPos).mag();

        for (int i = 0; i < Tuning.Shooter.SHOOTER_TABLE.length - 1; i++) {
            if (Tuning.Shooter.SHOOTER_TABLE[i].getDist() < distance && distance < Tuning.Shooter.SHOOTER_TABLE[i+1].getDist()) {

                // amount that distance is from first to second distance
                double t = (distance - Tuning.Shooter.SHOOTER_TABLE[i].getDist()) / (Tuning.Shooter.SHOOTER_TABLE[i+1].getDist() - Tuning.Shooter.SHOOTER_TABLE[i].getDist());
                
                double baseFlywheelSpeed = Tuning.Shooter.SHOOTER_TABLE[i].getSpeed();
                double speed = baseFlywheelSpeed + (Tuning.Shooter.SHOOTER_TABLE[i+1].getSpeed() - Tuning.Shooter.SHOOTER_TABLE[i].getSpeed()) * t;
                double baseAngle = Tuning.Shooter.SHOOTER_TABLE[i].getAngle();
                double angle = baseAngle + (Tuning.Shooter.SHOOTER_TABLE[i+1].getAngle() - Tuning.Shooter.SHOOTER_TABLE[i].getAngle()) * t;
                
                shooter.setTargetSpeed(speed);
                shooter.setTargetAngle(angle - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        shooter.setTargetAngle(0);
        shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
    }

}