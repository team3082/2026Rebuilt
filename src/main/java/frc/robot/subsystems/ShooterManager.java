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
                // When idle, hood goes down to fit under trench
                shooter.setTargetAngle(0);
                break;

            case ZEROING:
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

    public static void stopShooting() {
        shooterState = ShooterState.IDLE;
    }

    public static void zeroTurret() {
        turret.zero();
    }

    public static void zeroHood() {
        shooterState = ShooterState.ZEROING;
    }

    private static void aimTurret() { 
        // Change position for the fuel
        Vector2 deltaPos = target.pos.sub(Odometry.getPosition());

        // gets angle that turret need to relative to robot
        double targetTurretAngle = turret.clampAngle(Math.atan2(deltaPos.x, deltaPos.y) + Pigeon.getRotationRad());

        // if desired turret position is in the hard stop, rotate the robot so turret can aim at target

        if (targetTurretAngle == -10) {
            targetTurretAngle = Constants.Shooter.TURRET_MIN_ANGLE;
            // checks that driver is not rotating so rotation doesn't override their input
            if (SwerveManager.rotationSpeed == 0 && (shooterState == ShooterState.REVVING || shooterState == ShooterState.SHOOTING)) {
                SwerveManager.rotateAndDrive(0.4, SwerveManager.movement);
            }
        }

        turret.setAngle(targetTurretAngle);
    }

    private static void setShooterAngleAndSpeed() {
        
        switch (target) {
            // Seperates shooting targets because hub and passing need different tuned value
            case HUB:
                aimAtHub();
                break;
        
            default:
                aimPass();
                break;
        }

    }

    private static void aimPass() {
        Vector2 turretPos = Odometry.getPosition().add(Constants.Shooter.TURRET_POS_OFFSET.rotate(Pigeon.getRotationRad()));

        // gets distance between turret and target
        double distance = target.pos.sub(turretPos).mag();

        // based on distance, uses shooter table to set flywheel speeds for different ranges of distances
        for (int i = Tuning.Shooter.SHOOTER_TABLE_PASSING.length - 1; i >= 0; i--) {
            if (Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getDist() < distance) {
                shooter.setTargetSpeed(Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getSpeed());
                shooter.setTargetAngle(Tuning.Shooter.SHOOTER_TABLE_PASSING[i].getAngle() - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        shooter.setTargetAngle(0);
        shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
    }

    private static void aimAtHub() {

        Vector2 turretPos = Odometry.getPosition().add(Constants.Shooter.TURRET_POS_OFFSET.rotate(Pigeon.getRotationRad()));

        // gets distance between turret and hub
        double distance = target.pos.sub(turretPos).mag();

        for (int i = 0; i < Tuning.Shooter.SHOOTER_TABLE_HUB.length - 1; i++) {
            // finds which two values current distance is between and interpolates hood angle and flywheel speed between those values
            // if value is greater than maximum distance, continues linear approximation to that distance
            if ((Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist() < distance && distance < Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getDist()) || i == Tuning.Shooter.SHOOTER_TABLE_HUB.length - 2) {

                // amount that distance is from first to second distance
                double t = (distance - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist()) / (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getDist() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getDist());
                
                // interpolates flywheel speed and angle
                double baseFlywheelSpeed = Tuning.Shooter.SHOOTER_TABLE_HUB[i].getSpeed();
                double speed = baseFlywheelSpeed + (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getSpeed() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getSpeed()) * t;
                double baseAngle = Tuning.Shooter.SHOOTER_TABLE_HUB[i].getAngle();
                double angle = baseAngle + (Tuning.Shooter.SHOOTER_TABLE_HUB[i+1].getAngle() - Tuning.Shooter.SHOOTER_TABLE_HUB[i].getAngle()) * t;
                
                shooter.setTargetSpeed(speed);
                shooter.setTargetAngle(angle - Constants.Shooter.HOOD_ANGLE_OFFSET);
                return;
            }
        }

        // if not in range, sets angle to 0 and default flywheel speed
        shooter.setTargetAngle(0);
        shooter.setTargetSpeed(1000);
        System.out.println("Can't shoot from here");
    }

}