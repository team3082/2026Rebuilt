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
        switch (ShooterResults) {
            case AIMING:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.YELLOW_CYAN);
                } else {
                    LEDManager.setColor(Colors.YELLOW);
                }
                break;
            case REVVING:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.YELLOW_CYAN_SCROLL);
                } else {
                    LEDManager.setColor(Colors.YELLOW_SCROLL);
                }
                break;
            case SHOOTING:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.GREEN_CYAN_SCROLL);
                } else {
                    LEDManager.setColor(Colors.GREEN_SCROLL);
                }
                break;
            case ALL_CLEAR:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.GREEN_CYAN);
                } else {
                    LEDManager.setColor(Colors.GREEN);
                }
                break;
            case TOO_CLOSE:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.RED_CYAN);
                } else {
                    LEDManager.setColor(Colors.RED);
                }
                break;
            case CANT_AIM:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.RED_CYAN_SCROLL);
                } else {
                    LEDManager.setColor(Colors.RED_SCROLL);
                }
                break;
            case TRENCH:
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.MAGENTA_CYAN);
                } else {
                    LEDManager.setColor(Colors.MAGENTA);
                }
                break;
            case ZEROING: //to be deleted
                if (Intake.getIntakeState() == IntakeState.INTAKING) {
                    LEDManager.setColor(Colors.WHITE_CYAN);
                } else {
                    LEDManager.setColor(Colors.WHITE);
                }
                break;
            default:
                break;
        }
        
        }
    }
}
