package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.Tuning;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterState;

public class ShooterVisualizer {
    // Hood and flywheel (side view)
    private static MechanismRoot2d hoodRoot;
    private static MechanismLigament2d hoodLig;
    private static WheelMech flywheel;

    // Intake visualization 
    private static MechanismRoot2d intakeRoot;
    private static MechanismLigament2d intakeLig;
    private static WheelMech intakeWheel;   

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
    private static double spindleAngle;

    public static void init(){
        // Intake visualization (side view)
        intakeRoot = Telemetry.subsystemViewSide.getRoot("Intake", 3, 1.5);
        intakeLig = intakeRoot.append(new MechanismLigament2d("intakeLig", 2, 135));
        intakeWheel = new WheelMech("Intake Wheel", intakeLig, 6, .35, 5, new Color8Bit(Color.kDarkOrange));

        // Spindle/Indexer side view - coin flip style
        spindleSideRoot = Telemetry.subsystemViewSide.getRoot("SpindleIndexer", 4.2, 2);
        spindleFrontLig = spindleSideRoot.append(new MechanismLigament2d("spindleFront", 0.8, 0));
        spindleFrontLig.setColor(new Color8Bit(Color.kOrange));
        spindleBackLig = spindleSideRoot.append(new MechanismLigament2d("spindleBack", 0.8, 180));
        spindleBackLig.setColor(new Color8Bit(Color.kDarkOrange));
        
        // Turret side view - coin flip style
        turretSideRoot = Telemetry.subsystemViewSide.getRoot("TurretRight", 6.4, 2.4);
        turretFrontLig = turretSideRoot.append(new MechanismLigament2d("turretFront", 1, 0));
        turretBackLig = turretSideRoot.append(new MechanismLigament2d("turretBack", 1, 180));
        turretBackLig.setColor(new Color8Bit(Color.kOrange));

        // Hood and flywheel visualization (side view)
        hoodRoot = Telemetry.subsystemViewSide.getRoot("shooterRoot", 6.4-1, 2.6);
        hoodLig = hoodRoot.append(new MechanismLigament2d("hoodLig", 2, 0));
        flywheel = new WheelMech("Fly Wheel", hoodLig, 9, .35, 7.5, new Color8Bit(Color.kDarkOrange));

        // Turret top view
        turretRoot = Telemetry.subsystemViewTop.getRoot("turretRoot", 2.5, 2.5);
        turretLig = turretRoot.append(new MechanismLigament2d("turretLig", 1, 0));
        turretBack = turretRoot.append(new MechanismLigament2d("TurretBackArrow ", 1, 180));
        
        turretLig.append(new MechanismLigament2d("ArrowLeft", .8, 150+90-30));
        turretLig.append(new MechanismLigament2d("ArrowRight", .8, 30+90+30));

    }

    public static void update(){
        hoodAngle = ShooterManager.getShooter().getAngle();
        flywheelSpeed = ShooterManager.getShooter().getVelocity();
    
        hoodLig.setAngle(Math.toDegrees(hoodAngle));
        flywheel.update(flywheelSpeed * 0.01);

        //Intake 
        double intakeAngle = Constants.Intake.INTAKE_DOWN_ANGLE + 200;
        double intakeWheelSpeed = Intake.getIntakeState() == Intake.IntakeState.INTAKING ? Tuning.Intake.SPEED : 0;

        intakeLig.setAngle(intakeAngle);
        intakeWheel.update(intakeWheelSpeed);

        double turretAngle = ShooterManager.getTurret().getAngle();
        double turretRotation = -Math.toDegrees(turretAngle - Pigeon.getRotationRad());

        turretLig.setAngle(turretRotation+90);
        turretBack.setAngle(turretRotation+90-180);
        
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

        spindleAngle += ShooterManager.getShooterState() == ShooterState.SHOOTING ? .05 : 0;
        
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

    }
}