package frc.robot.subsystems.visualizer;


import java.util.PrimitiveIterator;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Telemetry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterVisualizer {
private static MechanismRoot2d hoodRoot;
private static MechanismLigament2d hoodLig;

private static MechanismLigament2d flywheelLig1;
private static MechanismLigament2d flywheelLig2;
private static MechanismLigament2d flywheelLig3;
private static MechanismLigament2d flywheelLig4;

private static MechanismRoot2d turretRoot;
private static MechanismLigament2d turretLig;



private static double hoodAngle;
private static double flywheelAngle;
private static double flywheelSpeed;


    
    public static void init(){
        
        hoodRoot = Telemetry.subsystemView.getRoot("shooterRoot", 1, 1);
        hoodLig = hoodRoot.append(new MechanismLigament2d("hoodLig", 2, 0));
        flywheelLig1 = hoodLig.append(new MechanismLigament2d("flywheelLig1", .5, 0));
        flywheelLig2 = hoodLig.append(new MechanismLigament2d("flywheelLig2", .5, 90));
        flywheelLig3 = hoodLig.append(new MechanismLigament2d("flywheelLig3", .5, 180));
        flywheelLig4 = hoodLig.append(new MechanismLigament2d("flywheelLig4", .5, 270));

        turretRoot = Telemetry.subsystemView.getRoot("turretRoot", 1, 3);
        turretLig = turretRoot.append(new MechanismLigament2d("turretLig", 1, 0));
    }

    public static void update(){
        hoodAngle = Shooter.getAngle();
        flywheelSpeed = Shooter.getVelocity();
    
        hoodLig.setAngle(hoodAngle);
        flywheelAngle += flywheelSpeed * 0.1;
        flywheelLig1.setAngle(flywheelAngle);
        flywheelLig2.setAngle(flywheelAngle + 90);
        flywheelLig3.setAngle(flywheelAngle + 180);
        flywheelLig4.setAngle(flywheelAngle + 270);

        turretLig.setAngle(Turret.getAngle());
    }
}
