package frc.robot.subsystems.LEDs;

import frc.robot.subsystems.LEDs.LEDManager.Colors;
import frc.robot.vision.VisionManager;

public class LEDsPreGame {
    public static void update(){
    
        if (LEDManager.currGameState == LEDManager.GameState.PREGAME){
            
            
            if (VisionManager.seeAprilTag == true){
                LEDManager.setColor(Colors.GREEN);
            } else if (VisionManager.cameraNum >= 1){
                LEDManager.setColor(Colors.YELLOW);
            } else {
                LEDManager.setColor(Colors.RED);
            }
        }
    
    }
}
