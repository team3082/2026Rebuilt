package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDMech2D {
    private static Mechanism2d mech2dLED = new Mechanism2d(60, 10);
    private static MechanismRoot2d root = mech2dLED.getRoot("root1", 5, 5);
    private static MechanismLigament2d ligs[] = new MechanismLigament2d[LEDManager.m_ledBuffer.getLength()];

    public static void init(){
        for (int i = 0; i<ligs.length; i++){
            ligs[i] = new MechanismLigament2d("led", 50/ligs.length, 0);
        }
        root.append(ligs[0]);
        for (int i = 1; i<ligs.length; i++){
            ligs[i-1].append(ligs[i]);
        }
        SmartDashboard.putData("LEDs", mech2dLED);
        //Shuffleboard.getTab("Robot Views").add("LEDs", mech2dLED);
    }

    public static void update(){
        for (int i = 0; i<ligs.length; i++){
            ligs[i].setColor(new Color8Bit(LEDManager.m_ledBuffer.getRed(i),LEDManager.m_ledBuffer.getBlue(i),LEDManager.m_ledBuffer.getGreen(i)));
        }
    }
}
