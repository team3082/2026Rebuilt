package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDMech2D {
    private static Mechanism2d mech2dLED = new Mechanism2d(10, 10);
    private static MechanismRoot2d root = mech2dLED.getRoot("root1", 5, 5);
    private static MechanismLigament2d lig1 = root.append(
        new MechanismLigament2d (
            "lig1", 
            0.5, 
            90)
    );

    public static void mechLED(Color8Bit color){
        lig1.setColor(color);
        SmartDashboard.putData("Mech2d", mech2dLED);
    }

}
