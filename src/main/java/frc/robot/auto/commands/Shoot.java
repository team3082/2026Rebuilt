package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class Shoot extends Command{

    private double lastShotTime;
    private boolean reachedShooting; 
    
    @Override
    public void initialize() {
        System.out.println("shooting");
        lastShotTime = RTime.now();
        ShooterManager.shoot();
    }

    @Override
    public void execute() {
        ShooterTarget target = ShooterManager.getTarget();
        Vector2 shotAim = target.pos.sub(Odometry.getPosition());
        double targetAngle = Math.atan2(shotAim.y, shotAim.x) + Math.PI;

        SwervePID.setDestState(Odometry.getPosition(), targetAngle);
        SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());

        if (ShooterManager.getShooterState() == ShooterState.SHOOTING) {
            reachedShooting = true;
        }

        Intake.startFeeding(.9);

        if (!reachedShooting || Shooter.getVelocity() < Shooter.getTargetSpeed() - Constants.Shooter.RPM_DROP) {
            lastShotTime = RTime.now(); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        ShooterManager.stopShooting();
        Intake.startIntaking();
    }

    @Override
    public boolean isFinished() {
        return RTime.now() - lastShotTime > Constants.Shooter.BALL_TIMEOUT; 
    }

}
