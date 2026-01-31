package frc.robot.subsystems.scoring.shooter;

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

                switch (target) {
                    case HUB:

                        break;

                    case PASS_LEFT:

                        break;

                    case PASS_RIGHT:

                        break;
                }
                break;

            case SHOOTING:
                Shooter.shooterState = ShooterState.SHOOTING;

                break;
        }
    }

}