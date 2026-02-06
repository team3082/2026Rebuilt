package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.states.ShooterState;

public class ShooterVisualizer {
    // Hood and flywheel (side view)
    private static MechanismRoot2d hoodRoot;
    private static MechanismLigament2d hoodLig;
    private static WheelMech mech;

    // Turret rotation (top view)
    private static MechanismRoot2d turretRoot;
    private static MechanismLigament2d turretLig, turretBack;

    // Turret side view - coin flip animation
    private static MechanismRoot2d turretSideRoot;
    private static MechanismLigament2d turretFrontLig, turretBackLig;

    // Spindle/Indexer side view - coin flip animation
    private static MechanismRoot2d spindleSideRoot;
    private static MechanismLigament2d spindleFrontLig, spindleBackLig;

    private static double hoodAngle;
    private static double flywheelSpeed;
    private static double angle;
    private static double spindleAngle;

    public static void init(){
        // Hood and flywheel visualization (side view)
        hoodRoot = Telemetry.subsystemViewSide.getRoot("shooterRoot", 2.5, 2.2);
        hoodLig = hoodRoot.append(new MechanismLigament2d("hoodLig", 2, 0));
        mech = new WheelMech("Fly Wheel", hoodLig, 9, .35, 7.5, new Color8Bit(Color.kDarkOrange));

        // Turret side view - coin flip style
        turretSideRoot = Telemetry.subsystemViewSide.getRoot("TurretRight", 3.5, 2);
        turretFrontLig = turretSideRoot.append(new MechanismLigament2d("turretFront", 1, 0));
        turretBackLig = turretSideRoot.append(new MechanismLigament2d("turretBack", 1, 180));
        turretBackLig.setColor(new Color8Bit(Color.kOrange));

        // Turret top view
        turretRoot = Telemetry.subsystemViewTop.getRoot("turretRoot", 4, 4);
        turretLig = turretRoot.append(new MechanismLigament2d("turretLig", .5, 0));
        turretBack = turretRoot.append(new MechanismLigament2d("TurretBackArrow ", .5, 180));
        
        turretLig.append(new MechanismLigament2d("ArrowLeft", .4, 150+90-30));
        turretLig.append(new MechanismLigament2d("ArrowRight", .4, 30+90+30));

        // Spindle/Indexer side view - coin flip style
        spindleSideRoot = Telemetry.subsystemViewSide.getRoot("SpindleIndexer", 1.5, 1.5);
        spindleFrontLig = spindleSideRoot.append(new MechanismLigament2d("spindleFront", 0.8, 0));
        spindleFrontLig.setColor(new Color8Bit(Color.kOrange));
        spindleBackLig = spindleSideRoot.append(new MechanismLigament2d("spindleBack", 0.8, 180));
        spindleBackLig.setColor(new Color8Bit(Color.kDarkOrange));
    }

    public static void update(){
        hoodAngle = Shooter.getAngle();
        flywheelSpeed = Shooter.getVelocity();
    
        hoodLig.setAngle(hoodAngle * 180 / Math.PI);
        mech.update(flywheelSpeed * 0.01);

        double turretAngle = Turret.getAngle();
        double turretRotation = turretAngle * Constants.RAD_TO_DEGREES;
        
        double turretCos = Math.cos(Math.toRadians(turretRotation));
        double turretWidth = Math.abs(turretCos);
        
        turretFrontLig.setLength(turretWidth);
        turretBackLig.setLength(turretWidth);
        
        if (turretCos > 0) {
            turretFrontLig.setAngle(0);
            turretBackLig.setAngle(180);
        } else {
            turretFrontLig.setAngle(180);
            turretBackLig.setAngle(0);
        }

        spindleAngle +=  ShooterManager.shooterState == ShooterState.SHOOTING ? 0.05 : 0;
        
        double spindleCos = Math.cos(spindleAngle);
        double spindleWidth = Math.abs(spindleCos);
        
        spindleFrontLig.setLength(spindleWidth);
        spindleBackLig.setLength(spindleWidth);
        
        if (spindleCos > 0) {
            spindleFrontLig.setAngle(0);
            spindleBackLig.setAngle(180);
        } else {
            spindleFrontLig.setAngle(180);
            spindleBackLig.setAngle(0);
        }

        turretLig.setAngle(turretRotation+90);
        turretBack.setAngle(turretRotation+90-180);
    }
}