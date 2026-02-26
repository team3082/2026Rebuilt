package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command{
    
    public void initialize() {
        Intake.startIntaking();
    }

    public void end() {
        Intake.stopIntaking();
    }
}
