

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Telemetry;

/** Add your docs here. */
public class IntakeVisualizer {
    private static LoggedMechanismRoot2d root;
    private static LoggedMechanismLigament2d lig1, lig2, lig3, lig4;
    private static double angle;

    public static void init(){
        root = Telemetry.subsystemView.getRoot("intakeRoot", 2, 2);
        lig1 = root.append(new LoggedMechanismLigament2d("lig1", 1, 0));
        lig2 = root.append(new LoggedMechanismLigament2d("lig2", 1, 0));
        lig3 = root.append(new LoggedMechanismLigament2d("lig3", 1, 0));
        lig4 = root.append(new LoggedMechanismLigament2d("lig4", 1, 0));

    }

    public static void update(){
        angle += Intake.getSpeed()*2;

        lig1.setAngle(angle+0);
        lig2.setAngle(angle+90);
        lig3.setAngle(angle+180);
        lig4.setAngle(angle+270);
    }
}
