package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.vision.AprilTag;
import frc.robot.utils.Vector2;

public class Constants {

    public static final class Swerve {
        public static final double PERCENT_OUT_TO_MOVE_VEL = 16 * 12;
        public static final double PERCENT_OUT_TO_ROTATE_VEL = 1; 

        public static final int DRIVEID0 = 6;
        public static final int DRIVEID1 = 8;
        public static final int DRIVEID2 = 2;
        public static final int DRIVEID3 = 4;

        public static final int STEERID0 = 5;
        public static final int STEERID1 = 7;
        public static final int STEERID2 = 1;
        public static final int STEERID3 = 3;

        public static final double MODOFFSET0 = 0.801514;
        public static final double MODOFFSET1 = 0.221191;
        public static final double MODOFFSET2 = 0.127197;
        public static final double MODOFFSET3 = 0.115479;

        public static final double DRIVE_RATIO = 6.12;
        public static final double STEER_RATIO = 150.0 /  7.0;


        public static final double WIDTH = 27.5;
        public static final double LENGTH = 27.5;
        public static final double MODULEOFFSET = 2.625;

        public static final double SWERVEMODX0 = Math.abs((WIDTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODX1 = -Math.abs((WIDTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODX2 = -Math.abs((WIDTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODX3 = Math.abs((WIDTH / 2.0) - MODULEOFFSET);

        public static final double SWERVEMODY0 = -Math.abs((LENGTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODY1 = -Math.abs((LENGTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODY2 = Math.abs((LENGTH / 2.0) - MODULEOFFSET);
        public static final double SWERVEMODY3 = Math.abs((LENGTH / 2.0) - MODULEOFFSET);

        public static final double driveTrackwidth = 0.0;
        public static final double driveWheelbase = 0.0;

        public static final double shootWhileMoveSpeed = 0.24; // ESSENTIAL DO NOT DELETE
    
        // The unadjusted maximum velocity of the robot, in inches per second.
        public static final double maxChassisVelocity = 6380.0 / 60.0 * 6.12 * (4.0 * Math.PI);
        // The unadjusted theoretical maximum angular velocity of the robot, in radians per second.
        public static final double maxAngularVelocity = (maxChassisVelocity / Math.hypot(driveTrackwidth / 2.0, driveWheelbase / 2.0));

        // Rotations to Radians
        public static final double rotConversionFactor = 2 * Math.PI;

        public static final class MK4iDriveRatios {
            // SDS MK4i - (8.14 : 1)
            public static final double L1 = (8.14 / 1.0);
            // SDS MK4i - (6.75 : 1)
            public static final double L2 = (6.75 / 1.0);
            // SDS MK4i - (6.12 : 1)
            public static final double L3 = (6.12 / 1.0);
        }

        public static final class MK4DriveRatios {
            // SDS MK4 - (8.14 : 1)
            public static final double L1 = (8.14 / 1.0);
            // SDS MK4 - (6.75 : 1)
            public static final double L2 = (6.75 / 1.0);
            // SDS MK4 - (6.12 : 1)
            public static final double L3 = (6.12 / 1.0);
            // SDS MK4 - (5.14 : 1)
            public static final double L4 = (5.14 / 1.0);
        }

        public static final class MK4iConstants {

            // Swerve module's wheel diameter
            public static final double wheelDiameter = 4;
            // The gear ratio of the steering motor
            public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

            // Whether or not, based on our module, the motors should be inverted
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

            // Should the CANCoder's magnet be inverted as well?
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        }

        public static final class MK4Constants {
        
            // Swerve module's wheel diameter
            public static final double wheelDiameter = 4;

            // The gear ratio of the steering motor
            public static final double angleGearRatio = (12.8 / 1.0);

            // Whether or not, based on our module, the motors should be inverted
            public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;

            // Should the CANCoder's magnet be inverted as well?
            public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        }

    }
    
    public static final double METERSTOINCHES = 39.3701;
    public static final double FIELD_HEIGHT = 317.7;
    public static final double FIELD_WIDTH = 651.2; 

    public static final boolean REPLAY = false;
    public static final double ROTATAIONAL_DEADBAND = 0;

    public static final class Shooter {

        public static final int TURRET_MOTOR_ID = 0;
        public static final int FLYWHEEL_MOTOR_ID = 0;
        public static final int HOOD_MOTOR_ID = 0;
        public static final int HALL_EFFECT_SENSOR_ID = 0;
        
        public static final double TURRET_GEAR_RATIO = 200.0 / 20.0; 
        public static final double HOOD_GEAR_RATIO = 380.0 / 20.0 * 24.0 / 15.0 * 4.0;

        public static final double FLYWHEEL_DIAMETER = 4.0;

        public static final double TURRET_ZERO_ANGLE = 0.0;
        public static final double TURRET_MIN_ANGLE = Math.toRadians(0.0);
        public static final double TURRET_MAX_ANGLE = Math.toRadians(300.0);

        public static final double HOOD_MAX_ANGLE = Math.toRadians(35.0);
        public static final double HOOD_ANGLE_OFFSET = Math.toRadians(25.0);

        public static final Vector2 TURRET_POS_OFFSET = new Vector2(3.75, 4.25);

    }

    public static final class Intake {
        public static final int PIVOT_MOTOR_ID = 0;
        public static final int ROLLER_MOTOR_ID = 0;

        public static final double INTAKE_DOWN_ANGLE = 0.0;
    }

    public static final class Indexer {
        public static final int SPINDEXER_ID = 0;
        public static final int HANDOFF_ID = 0;
    }
    public static final AprilTag[] APRIL_TAGS = {
        null,
        // Red Alliance
                 // id 1
        new AprilTag(1, (467.64), (292.31), (35.00), Math.toRadians(180), 0),
        new AprilTag(2, (469.11), (182.60), (44.25), Math.toRadians(90), 0),
        new AprilTag(3, (445.35), (172.84), (44.25), Math.toRadians(180), 0),
        new AprilTag(4, (445.35), (158.84), (44.25), Math.toRadians(180), 0),
        new AprilTag(5, (469.11), (135.09), (44.25), Math.toRadians(270), 0),
        new AprilTag(6, (467.64), (25.37), (35.00), Math.toRadians(180), 0),
        new AprilTag(7, (470.59), (25.37), (35.00), Math.toRadians(0), 0),
        new AprilTag(8, (483.11), (135.09), (44.25), Math.toRadians(270), 0),
        new AprilTag(9, (492.88), (144.84), (44.25), Math.toRadians(0), 0),
        new AprilTag(10, (492.88), (158.84), (44.25), Math.toRadians(0), 0),
        new AprilTag(11, (483.11), (182.60), (44.25), Math.toRadians(90), 0),
        new AprilTag(12, (470.59), (292.31), (35.00), Math.toRadians(0), 0),
        new AprilTag(13, (650.92), (291.47), (21.75), Math.toRadians(180), 0),
        new AprilTag(14, (650.92), (274.47), (21.75), Math.toRadians(180), 0),
        new AprilTag(15, (650.90), (170.22), (21.75), Math.toRadians(180), 0),
        new AprilTag(16, (650.90), (153.22), (21.75), Math.toRadians(180), 0),
        // Blue Team
        new AprilTag(17, (183.59), (25.37), (35.00), Math.toRadians(0), 0),
        new AprilTag(18, (182.11), (135.09), (44.25), Math.toRadians(270), 0),
        new AprilTag(19, (205.87), (144.84), (44.25), Math.toRadians(0), 0),
        new AprilTag(20, (205.87), (158.84), (44.25), Math.toRadians(0), 0),
        new AprilTag(21, (182.11), (182.60), (44.25), Math.toRadians(90), 0),
        new AprilTag(22, (183.59), (292.31), (35.00), Math.toRadians(0), 0),
        new AprilTag(23, (180.64), (292.31), (35.00), Math.toRadians(180), 0),
        new AprilTag(24, (168.11), (182.60), (44.25), Math.toRadians(90), 0),
        new AprilTag(25, (158.34), (172.84), (44.25), Math.toRadians(180), 0),
        new AprilTag(26, (158.34), (158.84), (44.25), Math.toRadians(180), 0),
        new AprilTag(27, (168.11), (135.09), (44.25), Math.toRadians(270), 0),
        new AprilTag(28, (180.64), (25.37), (35.00), Math.toRadians(180), 0),
        new AprilTag(29, (0.30), (26.22), (21.75), Math.toRadians(0), 0),
        new AprilTag(30, (0.30), (43.22), (21.75), Math.toRadians(0), 0),
        new AprilTag(31, (0.32), (147.47), (21.75), Math.toRadians(0), 0),
        new AprilTag(32, (0.32), (164.47), (21.75), Math.toRadians(0), 0),
        
    };
}