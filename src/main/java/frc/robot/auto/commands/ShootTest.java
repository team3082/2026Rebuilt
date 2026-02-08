package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.utils.RTime;

public class ShootTest extends Command{
    public double duration = 5;
    private double startTime;

    @Override
    public void initialize() {
        ShooterManager.setTarget(ShooterTarget.HUB);
        ShooterManager.shoot();
        startTime = RTime.now();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return RTime.now() - startTime >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        ShooterManager.shoot();
    }
    
}
