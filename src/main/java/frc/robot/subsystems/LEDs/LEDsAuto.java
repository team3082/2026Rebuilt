package frc.robot.subsystems.LEDs;

import frc.robot.subsystems.LEDs.LEDManager.Colors;

public class LEDsAuto {
    public static void update(){
    
        if (LEDManager.currGameState == LEDManager.GameState.AUTO || true){
            
            LEDManager.setColor(Colors.RED_SCROLL);
            
        }
    
    }
}
