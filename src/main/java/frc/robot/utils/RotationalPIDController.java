package frc.robot.utils;

public class RotationalPIDController extends PIDController {

    public RotationalPIDController(double p, double i, double d, double deadband, double velDeadband, double maxOutput) {
        super(p, i, d, deadband, velDeadband, maxOutput);
    }

    @Override
    public void setDest(double dest) {
        super.setDest(dest); // no need to modulo — atan2 handles it
    }

    @Override
    public double updateOutput(double pos) {
        double error = Math.atan2(Math.sin(pos - dest), Math.cos(pos - dest));
        return super.updateOutput(dest + error); // shift pos by clean error around dest
    }
}