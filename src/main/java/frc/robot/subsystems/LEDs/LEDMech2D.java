package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDMech2D {
    private static Mechanism2d mech2dLED = new Mechanism2d(33.5, 10);
    private static MechanismRoot2d root = mech2dLED.getRoot("root1", 5, 5);
    private static MechanismLigament2d[] ligs = new MechanismLigament2d[47];
    public static void init(){
        SmartDashboard.putData("LED display", mech2dLED);
        for(int i = 0; i < 47; i++){
            ligs[i] = new MechanismLigament2d (
            "lig", 
            0.5, 
            0);
        }
        root.append(ligs[0]);
        for(int i = 1; i < 47; i++){
            ligs[i-1].append(ligs[i]);
        }
    }
    public static void update(){
        for(int i = 0; i < 47; i++){
            ligs[i].setColor(new Color8Bit(LEDManager.m_ledBuffer.getRed(i),LEDManager.m_ledBuffer.getBlue(i),LEDManager.m_ledBuffer.getGreen(i)));
        }
    }

}
