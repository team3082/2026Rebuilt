package frc.robot;

import frc.robot.subsystems.ShooterTableValue;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.QuadraticBezier;
import frc.robot.utils.trajectories.CubicBezier;
import frc.robot.utils.trajectories.Curve;
import frc.robot.utils.trajectories.LinearBezier;

public final class Tuning {
    //Swerve
    public static final double MOVEP = 0.0375;
    public static final double MOVEI = 0.0001;
    public static final double MOVED = 0.004;
    public static final double MOVEDEAD = 0.5;
    public static final double MOVEVELDEAD = 0.05;
    public static final double MOVEMAXSPEED = 0.3;
    
    public static final double ROTP = 0.4;
    public static final double ROTI = 0.05;
    public static final double ROTD = 0.075;
    public static final double ROTDEAD = 0.02;
    public static final double ROTVELDEAD = 0.05;
    public static final double ROTMAXSPEED = 0.3;
 
     //Tune
     public static final double SWERVE_TRJ_PPOS = 0.05;
     public static final double SWERVE_TRJ_IPOS = 0.00;
     public static final double SWERVE_TRJ_DPOS = 0.0002;

     public static final double SWERVE_TRJ_PROT = 0.225;
     public static final double SWERVE_TRJ_IROT = 0.0;
     public static final double SWERVE_TRJ_DROT = 0.01;

     public static final double SWERVE_KSPOS = 0.00;
     public static final double SWERVE_KVPOS = 0.85/160;
     public static final double SWERVE_KAPOS = 0.0007;

     public static final double SWERVE_KSROT = 0.00;//0.005;
     public static final double SWERVE_KVROT = 0.0;//0.55 / (3.0 * Math.PI);
     public static final double SWERVE_KAROT = 0.0;
 
     public static final int CURVE_RESOLUTION = 100;
     public static final double CURVE_DEADBAND = 0.5; // bro this is inches who had it at 0.001
     public static final double ROT_DEADBAND = 0.03; // radians
    

    // Trapezoidal Tuning
    public static final double MOVE_PRECISE_VEL = 16 * 12;
    public static final double MOVE_PRECISE_ACC = 8 * 12;
    public static final double MOVE_PRECISE_DEC = 8 * 12;

    public static final double MOVE_FAST_VEL = 16 * 12;
    public static final double MOVE_FAST_ACC = 10 * 12;
    public static final double MOVE_FAST_DEC = 10 * 12;

    public static final double ROT_PRECISE_VEL = 1.0;
    public static final double ROT_PRECISE_ACC = 0.5;
    public static final double ROT_PRECISE_DEC = 0.5;

    public static final double ROT_FAST_VEL = 1.0;
    public static final double ROT_FAST_ACC = 0.5;
    public static final double ROT_FAST_DEC = 0.5;

    public static final class OI {
        public static final double KDYAW = 0.00;

        public static final int YAWRATEFEEDBACKSTATUS = 0;

        public static final double NORMALSPEED = 1.0;

        public static final double ROTSPEED = 0.3;
    }

    public static final class Shooter {
        // Hood PID Constants
        public static final double HOOD_KP = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_KI = 0;
        public static final double HOOD_DEADBAND = 0;


        public static final double FLYWHEEL_SPEED_DEADBAND = 0.0;
        
        // Turret PID Constants
        public static final double TURRET_KP = 0.0;
        public static final double TURRET_KI = 0.0;
        public static final double TURRET_KD = 0.0;
        public static final double TURRET_DEADBAND = 2.0;
        
        // Turret Control
        public static final double TURRET_ZEROING_SPEED = -0.1;

        public static final ShooterTableValue[] SHOOTER_TABLE = { //TODO tune
            new ShooterTableValue(48.0, Math.toRadians(25.0), 2800),
            new ShooterTableValue(60.0, Math.toRadians(30.0), 3000),
            new ShooterTableValue(72.0, Math.toRadians(35.0), 3200),
            new ShooterTableValue(84.0, Math.toRadians(38.0), 3500),
            new ShooterTableValue(96.0, Math.toRadians(40.0), 3700),
            new ShooterTableValue(108.0, Math.toRadians(42.0), 3900),
            new ShooterTableValue(120.0, Math.toRadians(43.0), 4000),
            new ShooterTableValue(132.0, Math.toRadians(45.0), 4300)
        };
    }

    public static final class Indexer {
        public static final double SPINDEXER_SPEED = 0.0;
        public static final double HANDOFF_SPEED = 0.0;
    }

    public static final class AutoPaths {
      
        public static Curve getAutoPath(Curve path) {
            return path;
        }
    }
}