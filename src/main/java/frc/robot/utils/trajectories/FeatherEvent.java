package frc.robot.utils.trajectories;

import edu.wpi.first.wpilibj2.command.Command;

public class FeatherEvent {
    public double t;
    public Command command;

    public FeatherEvent(double t, Command command) {
        this.t = t;
        this.command = command;
    }
}
