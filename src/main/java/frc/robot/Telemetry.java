package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Constants.Intake;
import frc.robot.auto.Auto;
import frc.robot.subsystems.sensors.Pigeon;
// import frc.robot.subsystems.visualizer.IntakeVisualizer;
import frc.robot.subsystems.visualizer.ShooterVisualizer;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.swerve.visualizer.SwerveBaseVisualizer;
import frc.robot.swerve.SwervePID;

/*
 * handles telemetry for the robot
 * reads values from subsystems and updates tables and visualizers
 * includes the field view and subsystem views
 */
public class Telemetry {
    private static ShuffleboardTab robotTab = Shuffleboard.getTab("Robot Views");
    private static ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    // Views
    private static Field2d fieldView = new Field2d();
    public static Mechanism2d subsystemView = new Mechanism2d(120/Constants.METERSTOINCHES, 240/Constants.METERSTOINCHES);
    public static Mechanism2d subsytemView = new Mechanism2d(100, 100);
    public static Mechanism2d swerveView = new Mechanism2d(60, 60);

    // Logging
    // Swerve
    private static final GenericEntry SWERVE_MOD_1_ANGLE = swerveTab.add("Swerve Module 1 Angle", SwerveManager.mods[0].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_1_SPEED = swerveTab.add("Swerve Module 1 Speed", SwerveManager.mods[0].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_1_TARGET_ANGLE = swerveTab.add("Swerve Module 1 Target Angle", SwerveManager.mods[0].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_1_TARGET_SPEED = swerveTab.add("Swerve Module 1 Target Speed", SwerveManager.mods[0].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_1_INVERTED = swerveTab.add("Swerve Module 1 Inverted", SwerveManager.mods[0].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_2_ANGLE = swerveTab.add("Swerve Module 2 Angle", SwerveManager.mods[1].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_2_SPEED = swerveTab.add("Swerve Module 2 Speed", SwerveManager.mods[1].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_2_TARGET_ANGLE = swerveTab.add("Swerve Module 2 Target Angle", SwerveManager.mods[1].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_2_TARGET_SPEED = swerveTab.add("Swerve Module 2 Target Speed", SwerveManager.mods[1].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_2_INVERTED = swerveTab.add("Swerve Module 2 Inverted", SwerveManager.mods[1].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_3_ANGLE = swerveTab.add("Swerve Module 3 Angle", SwerveManager.mods[2].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_3_SPEED = swerveTab.add("Swerve Module 3 Speed", SwerveManager.mods[2].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_3_TARGET_ANGLE = swerveTab.add("Swerve Module 3 Target Angle", SwerveManager.mods[2].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_3_TARGET_SPEED = swerveTab.add("Swerve Module 3 Target Speed", SwerveManager.mods[2].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_3_INVERTED = swerveTab.add("Swerve Module 3 Inverted", SwerveManager.mods[2].inverted).getEntry();

    private static final GenericEntry SWERVE_MOD_4_ANGLE = swerveTab.add("Swerve Module 4 Angle", SwerveManager.mods[3].getSteerAngle()).getEntry();
    private static final GenericEntry SWERVE_MOD_4_SPEED = swerveTab.add("Swerve Module 4 Speed", SwerveManager.mods[3].getDriveVelocity()).getEntry();
    private static final GenericEntry SWERVE_MOD_4_TARGET_ANGLE = swerveTab.add("Swerve Module 4 Target Angle", SwerveManager.mods[3].targetAngle).getEntry();
    private static final GenericEntry SWERVE_MOD_4_TARGET_SPEED = swerveTab.add("Swerve Module 4 Target Speed", SwerveManager.mods[3].targetSpeed).getEntry();
    private static final GenericEntry SWERVE_MOD_4_INVERTED = swerveTab.add("Swerve Module 4 Inverted", SwerveManager.mods[3].inverted).getEntry();

    public static void init() {
        robotTab.add("Field View", fieldView);
        robotTab.add("Subsystem View", subsystemView);
        robotTab.add("Swerve View", swerveView);

        SwerveBaseVisualizer.init();
        ShooterVisualizer.init();
        robotTab.addString("Position", () -> SwervePosition.getPosition().toString());
        robotTab.addString("PID Dest Position", () -> SwervePID.getDest().toString());
        

        robotTab.add("Auto Selector", Auto.routineManager.autoSelector);

        robotTab.add("Subsystem Mech", subsystemView);
    }

    public static void update() {
        updateField();
        updateSwerve();
        logValues();
        ShooterVisualizer.update();
    }

    private static void logValues(){
        Logger.recordOutput("Robot/SwervePID/Error", SwervePID.getError().toString());
        Logger.recordOutput("Robot/SwervePID/Rot Error", SwervePID.getRotationError());
        Logger.recordOutput("Robot/SwervePID/At Dest", SwervePID.atDest());
        Logger.recordOutput("Robot/SwervePID/At Rot", SwervePID.atRot());
        Logger.recordOutput("Robot/Swerve Position", SwervePosition.getPosition().toString());
        Logger.recordOutput("Robot/Swerve Position/x", SwervePosition.getPosition().x);
        Logger.recordOutput("Robot/Swerve Position/y", SwervePosition.getPosition().y);
        Logger.recordOutput("Robot/Swerve Position/rot", Pigeon.getRotationRad());

    }

    private static void updateSwerve() {
        SWERVE_MOD_1_ANGLE.setDouble(SwerveManager.mods[0].getSteerAngle());
        SWERVE_MOD_1_SPEED.setDouble(SwerveManager.mods[0].getDriveVelocity());
        SWERVE_MOD_1_TARGET_ANGLE.setDouble(SwerveManager.mods[0].targetAngle);
        SWERVE_MOD_1_TARGET_SPEED.setDouble(SwerveManager.mods[0].targetSpeed);
        SWERVE_MOD_1_INVERTED.setBoolean(SwerveManager.mods[0].inverted);

        SWERVE_MOD_2_ANGLE.setDouble(SwerveManager.mods[1].getSteerAngle());
        SWERVE_MOD_2_SPEED.setDouble(SwerveManager.mods[1].getDriveVelocity());
        SWERVE_MOD_2_TARGET_ANGLE.setDouble(SwerveManager.mods[1].targetAngle);
        SWERVE_MOD_2_TARGET_SPEED.setDouble(SwerveManager.mods[1].targetSpeed);
        SWERVE_MOD_2_INVERTED.setBoolean(SwerveManager.mods[1].inverted);

        SWERVE_MOD_3_ANGLE.setDouble(SwerveManager.mods[2].getSteerAngle());
        SWERVE_MOD_3_SPEED.setDouble(SwerveManager.mods[2].getDriveVelocity());
        SWERVE_MOD_3_TARGET_ANGLE.setDouble(SwerveManager.mods[2].targetAngle);
        SWERVE_MOD_3_TARGET_SPEED.setDouble(SwerveManager.mods[2].targetSpeed);
        SWERVE_MOD_3_INVERTED.setBoolean(SwerveManager.mods[2].inverted);

        SWERVE_MOD_4_ANGLE.setDouble(SwerveManager.mods[3].getSteerAngle());
        SWERVE_MOD_4_SPEED.setDouble(SwerveManager.mods[3].getDriveVelocity());
        SWERVE_MOD_4_TARGET_ANGLE.setDouble(SwerveManager.mods[3].targetAngle);
        SWERVE_MOD_4_TARGET_SPEED.setDouble(SwerveManager.mods[3].targetSpeed);
        SWERVE_MOD_4_INVERTED.setBoolean(SwerveManager.mods[3].inverted);

        SwerveBaseVisualizer.update();
    }

    /**
     * Updates the simulated field in shuffleboard based on SwervePosition
     */
    private static void updateField() {
        // Current position adjusted to be in the center of the field at (0,0)

        int alliancePosMultiplier = 1;
        double allianceRotOffset = 0;
        if (!DriverStation.getAlliance().isEmpty()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                alliancePosMultiplier = -1;
                allianceRotOffset = Math.PI;
            }
        }

        // System.out.println("Alliance Pos Multiplier: " + alliancePosMultiplier + " Alliance Rot Offset: " + allianceRotOffset);
        
        Pose2d currentPose = new Pose2d(
            alliancePosMultiplier * SwervePosition.getPosition().x /Constants.METERSTOINCHES + 8.78,
            alliancePosMultiplier * SwervePosition.getPosition().y/Constants.METERSTOINCHES + 4.01,
            Rotation2d.fromRadians(Pigeon.getRotationRad() + Math.PI / 2.0 + allianceRotOffset)
        );
        fieldView.setRobotPose(currentPose);

        try {
        Logger.recordOutput("Robot/Swerve/Field Pose", currentPose);
        } catch (Exception e) {
            System.out.println("Oopsies!: " + e);
        }
    }

    

    
}
