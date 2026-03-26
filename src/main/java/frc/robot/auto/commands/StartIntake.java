package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StartIntake extends InstantCommand {
    
    public void initialize() {
        Intake.startIntaking();
    }

}
