package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.auto.ChickenPlannable;

public class Shoot extends Command {

    private double startTime;
    private final double duration;

    @ChickenPlannable
    public Shoot() {
        this(4.0); // Default 4 second duration
    }

    @ChickenPlannable
    public Shoot(double duration) {
        this.duration = duration;
    }

    @Override
    public void initialize() {
        startTime = RTime.now();
        ShooterManager.shoot();
    }

    @Override
    public void execute() {
        // Aiming and Swerve Control
        ShooterTarget target = ShooterManager.getTarget();
        Vector2 shotAim = target.pos.sub(Odometry.getPosition());
        double targetAngle = Math.atan2(shotAim.y, shotAim.x) + Math.PI;

        SwervePID.setDestState(Odometry.getPosition(), targetAngle);
        SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());

        // Feeding Logic
        if (ShooterManager.getShooterState() == ShooterState.SHOOTING) {
            // Oscillating feed speed
            double feedSpeed = Math.sin(Timer.getFPGATimestamp() * 10) * 0.5 + 0.5;
            Intake.startFeeding(feedSpeed);
        }

        System.out.println("TEST");
    }

    @Override
    public void end(boolean interrupted) {
        ShooterManager.stopShooting();
        Intake.startIntaking();
    }

    @Override
    public boolean isFinished() {
        // Command finishes strictly when the time duration has elapsed
        return (RTime.now() - startTime) > duration;
    }
}