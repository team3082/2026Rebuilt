package frc.robot.subsystems.LEDs;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.LEDs.LEDManager.Colors;
import frc.robot.subsystems.LEDs.LEDManager.GameState;
import frc.robot.subsystems.states.ShooterState;
import frc.robot.subsystems.states.ShooterTarget;

public class LEDsTeleop {
    
    public static void update(){
        if (LEDManager.currGameState == LEDManager.GameState.TELEOP){

        ShooterManager.LEDCheckReturn ShooterResults = ShooterManager.ledChecks();
        
        
        }
    }
}
