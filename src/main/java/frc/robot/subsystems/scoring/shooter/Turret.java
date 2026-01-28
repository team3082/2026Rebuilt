package frc.robot.subsystems.scoring.shooter;

import frc.robot.utils.PIDController;

public class Turret{
    private static double setAngle;
    private static PIDController turretPID;

    private static void init(){

    }

    private static void update(){

    }

    public static void setAngle(double angle){
        setAngle = angle;
    }

    /**
     * It checks if the turret base is within the selected angle.
     */
    public static boolean atAngle(){
        return true;
    }
}