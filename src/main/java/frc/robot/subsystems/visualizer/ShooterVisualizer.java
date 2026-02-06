package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterVisualizer {
    private static MechanismRoot2d hoodRoot;
    private static MechanismLigament2d hoodLig;

    private static WheelMech mech;

    private static MechanismRoot2d turretRoot;
    private static MechanismLigament2d turretLig;

    private static MechanismRoot2d turretSideRoot;
    private static MechanismLigament2d turretFrontLig, turretBackLig;

    private static double hoodAngle;
    private static double flywheelAngle;
    private static double flywheelSpeed;

    private static double angle;

    public static void init(){
        
        hoodRoot = Telemetry.subsystemViewSide.getRoot("shooterRoot", 1, 1);
        hoodLig = hoodRoot.append(new MechanismLigament2d("hoodLig", 2, 0));
        mech = new WheelMech("Fly Wheel", hoodLig, 9, .6, 5,  new Color8Bit(Color.kAliceBlue));

        turretSideRoot = Telemetry.subsystemViewSide.getRoot("TurretRight", 5, 2  );
        turretFrontLig = turretSideRoot.append(new MechanismLigament2d("asdsd", 1, 0));
        turretBackLig = turretSideRoot.append(new MechanismLigament2d("asdsdd", 1, 180));
        turretBackLig.setColor(new Color8Bit(Color.kBlanchedAlmond));

        turretRoot = Telemetry.subsystemViewTop.getRoot("turretRoot", 1, 3);
        turretLig = turretRoot.append(new MechanismLigament2d("turretLig", 1, 0));
    }

    public static void update(){
        hoodAngle = Shooter.getAngle();
        flywheelSpeed = Shooter.getVelocity();
        angle += 0.05;
    
        hoodLig.setAngle(hoodAngle * 180/Math.PI);
        mech.update(flywheelSpeed*.01);

        Turret.setAngle(angle / Constants.RAD_TO_DEGREES);
        turretLig.setAngle(Turret.getAngle() * Constants.RAD_TO_DEGREES);
        turretBackLig.setLength(Math.cos(Turret.getAngle()));
        turretFrontLig.setLength(Math.cos(-Turret.getAngle()));
    }
}
