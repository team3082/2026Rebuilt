package frc.robot.subsystems.visualizer;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Telemetry;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.states.ShooterState;

public class SubsystemVisualizer {
    // Hood and flywheel (side view)
    private static MechanismRoot2d hoodRoot;
    private static MechanismLigament2d hoodLig;
    private static WheelMech flywheel;

    // Intake visualization 
    private static MechanismRoot2d intakeRoot;
    private static MechanismLigament2d intakeLig;
    private static WheelMech intakeWheel;   

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
        
        // Hood and flywheel visualization (side view)
        hoodRoot = Telemetry.subsystemViewSide.getRoot("shooterRoot", 6.4-1, 2.6);
        hoodLig = hoodRoot.append(new MechanismLigament2d("hoodLig", 2, 0));
        flywheel = new WheelMech("Fly Wheel", hoodLig, 9, .35, 7.5, new Color8Bit(Color.kDarkOrange));

    }

    public static void update(){
        hoodAngle = Shooter.getAngle();
        flywheelSpeed = Shooter.getVelocity();
    
        hoodLig.setAngle(Math.toDegrees(hoodAngle));
        flywheel.update(flywheelSpeed * 0.01);

        //Intake 
        double intakeAngle = Intake.getAngle() + 200;
        double intakeWheelSpeed = Intake.getSpeed() * 7;

        intakeLig.setAngle(intakeAngle);
        intakeWheel.update(intakeWheelSpeed);

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