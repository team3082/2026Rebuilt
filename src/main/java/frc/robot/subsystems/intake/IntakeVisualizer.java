package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeVisualizer {
    private static MechanismRoot2d root;
    private static MechanismLigament2d arm, motor;
    private static Mechanism2d intakeSim;

    public static boolean motorSpinning;

    public static void init(){
        motorSpinning = false;
        intakeSim = new Mechanism2d(100, 100);
        root = intakeSim.getRoot("intakeRoot", 50, 50);
        arm = root.append(new MechanismLigament2d("Intake Arm", 25, 0));
        motor = arm.append(new MechanismLigament2d("Intake Motor", 7, 45));

        SmartDashboard.putData("Intake Sim", intakeSim);

        arm.setAngle(0);
        motor.setAngle(0);
        
    }

    // updates angles in sim 
    public static void update(){
        arm.setAngle(Intake.currentAngle * (180 / Math.PI));
        if(motorSpinning){
            motor.setAngle(motor.getAngle()+30);
        } 
    }
}
