package frc.robot;

import frc.robot.subsystems.ShooterTableValue;
import frc.robot.utils.Vector2;

public final class Tuning {
    //Swerve
    public static final double MOVEP = 0.0375;
    public static final double MOVEI = 0.0001;
    public static final double MOVED = 0.004;
    public static final double MOVEDEAD = 1;
    public static final double MOVEVELDEAD = 0.05;
    public static final double MOVEMAXSPEED = 0.3;
    
    public static final double ROTP = 0.4;
    public static final double ROTI = 0.05;
    public static final double ROTD = 0.075;
    public static final double ROTDEAD = 0.5;
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
    public static final double ROT_DEADBAND = 0.005; // radians
    
    // Holonomic Drive Controller
    public static final double holonomic_pos_kp = 0.05;
    public static final double holonomic_pos_ki = 0.00001;
    public static final double holonomic_pos_kd = 0.001;

    public static final double holonomic_rot_kp = 0.45;
    public static final double holonomic_rot_ki = 0.03;
    public static final double holonomic_rot_kd = 0.06;


    public static final double holonomic_vel_kp = 0.05;
    public static final double holonomic_vel_ki = 0.0;
    public static final double holonomic_vel_kd = 0.0;

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
        public static final double HOOD_KP = 0.05;
        public static final double HOOD_KD = 0;
        public static final double HOOD_KI = 0.000;
        public static final double HOOD_DEADBAND = 0.2;

        public static final double FLYWHEEL_P = 0.4;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.03;
        public static final double FLYWHEEL_KV = 0.12;

        public static final double FLYWHEEL_SPEED_DEADBAND = 100; // bro this is in rpm who set it to 0.1
        
        // zeroing
        public static final double HOOD_ZEROING_SPEED = -0.2;

        public static final ShooterTableValue[] SHOOTER_TABLE_HUB = {
            new ShooterTableValue(75.0, Math.toRadians(25.0), 2125),  //g
            new ShooterTableValue(90.0, Math.toRadians(25.0), 2150),  //g
            new ShooterTableValue(106.0, Math.toRadians(25.0), 2250), //g
            new ShooterTableValue(120.0, Math.toRadians(25.0), 2375), //g
            new ShooterTableValue(136.0, Math.toRadians(25.0), 2425), //g
            new ShooterTableValue(150.0, Math.toRadians(25.0), 2550), //g final rpm
            new ShooterTableValue(165.0, Math.toRadians(30.0), 2550), //g
            new ShooterTableValue(187.0, Math.toRadians(34.0), 2650), //prettygood
            new ShooterTableValue(220.0, Math.toRadians(38.0), 2700)  //g
        };

        public static final ShooterTableValue[] SHOOTER_TABLE_PASSING = {
            new ShooterTableValue(0.0, Math.toRadians(50.0), 800),
            new ShooterTableValue(60.0, Math.toRadians(50.0), 1000),
            new ShooterTableValue(120.0, Math.toRadians(50.0), 1300),
            new ShooterTableValue(180.0, Math.toRadians(50.0), 1600),
            new ShooterTableValue(240.0, Math.toRadians(50.0), 2000),
            new ShooterTableValue(300.0, Math.toRadians(50.0), 2300),
            new ShooterTableValue(360.0, Math.toRadians(50.0), 2600),
            new ShooterTableValue(420.0, Math.toRadians(50.0), 3000),
            new ShooterTableValue(480.0, Math.toRadians(50.0), 3300),
            new ShooterTableValue(540.0, Math.toRadians(50.0), 3600)
        };

        public static final double LOOK_AHEAD_TIME_K = 0.5;
        public static final Vector2 TURRET_POS_OFFSET = new Vector2(3.75, 4.25);
    }

    public static final class Indexer {
        public static final double SPINDEXER_SPEED = 0.55;
        public static final double HANDOFF_SPEED = -0.4;
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = -42.5;
        public static final double FEED_SPEED = -50;
        public static final double REVERSE_SPEED = 40;
        
        public static final double PIVOT_P = 0.45; 
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;
        public static final double PIVAT_KG = -0.18;

        // public static final double PIVOT_VEL = 1200;
        // public static final double PIVOT_ACCEL = 1600;
        // public static final double PIVOT_JERK = 1000;

        public static final double ROLLER_P = 0.01;
        public static final double ROLLER_I = 0;
        public static final double ROLLER_D = 0;
        public static final double ROLLER_KV = 0.13;
    }

    public static double holonomic_lookahead_time = 0.2;
}