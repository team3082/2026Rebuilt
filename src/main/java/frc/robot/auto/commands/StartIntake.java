package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.utils.auto.ChickenPlannable;

public class StartIntake extends InstantCommand {

    @ChickenPlannable
    public StartIntake(){}
    
    public void initialize() {
        Intake.startIntaking();
    }

}
