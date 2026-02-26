package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.utils.RTime;

public class Shoot extends Command{

    private double lastShotTime;
    private boolean reachedShooting; // starts detection of balls leaving once shooter starts
    
    @Override
    public void initialize() {
        lastShotTime = RTime.now();
        ShooterManager.shoot();
    }

    @Override
    public void execute() {
        if (ShooterManager.getShooterState() == ShooterState.SHOOTING) {
            reachedShooting = true;
        }

        if (!reachedShooting || ShooterManager.getShooter().getVelocity() < ShooterManager.getShooter().getTargetSpeed() - Constants.Shooter.RPM_DROP) {
            lastShotTime = RTime.now(); // resets time every time it shoots (rpm drops when a ball is shot)
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterManager.stopShooting();
    }

    @Override
    public boolean isFinished() {
        return RTime.now() - lastShotTime > Constants.Shooter.BALL_TIMEOUT; // checks if time since last shot is high enough to end
    }

}
