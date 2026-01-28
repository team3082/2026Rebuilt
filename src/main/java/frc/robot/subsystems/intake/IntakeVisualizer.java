

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Telemetry;

/** Add your docs here. */
public class IntakeVisualizer {
    private static MechanismRoot2d root;
    private static MechanismLigament2d Arm, Motor;
    private static double angle;
    private static Mechanism2d intakeSim;

    public static void init(){
        intakeSim = new Mechanism2d(100, 100);
        root = intakeSim.getRoot("intakeRoot", 50, 50);
        Arm = root.append(new MechanismLigament2d("Arm", 25, 0));
        Motor = Arm.append(new MechanismLigament2d("Motor", 7, 45));

        SmartDashboard.putData("Intake Sim", intakeSim);

        Arm.setAngle(0);
        Motor.setAngle(0);
        
    }

    public static void update(){

        Intake.startDown();

        if(Arm.getAngle() < Intake.getDownSpeed()){
            Arm.setAngle(Arm.getAngle()+0.01);
        }
        Motor.setAngle(Motor.getAngle()+10);



    }
}
