package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controllermaps.LogitechF310;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.RMath;
import frc.robot.utils.RTime;

public class OI {
    public static Joystick driverStick;

    // ------------------ Driver Controls ------------------ //

    // Movement
    public static final int moveX  = LogitechF310.AXIS_LEFT_X;
    public static final int moveY = LogitechF310.AXIS_LEFT_Y;
    public static final int rotateX  = LogitechF310.AXIS_RIGHT_X;
 
    // zero is for Pigeon 
    static final int zero = LogitechF310.BUTTON_Y;
 

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
    }

    public static void userInput() {
        driverInput();
    }

    /**
     * Instruct the robot to follow instructions from joysticks.
     * One call from this equals one frame of robot instruction.
     * Because we used TimedRobot, this runs 50 times a second,
     * so this lives in the teleopPeriodic() function.
     */
    public static void driverInput() {
        // INPUT

        // Reset pigeon
        if (driverStick.getRawButton(zero)) Pigeon.reset();

        /*--------------------------------------------------------------------------------------------------------*/
        // SETUP

        Vector2 drive = new Vector2(driverStick.getRawAxis(moveX) * Math.abs(driverStick.getRawAxis(moveX)), -driverStick.getRawAxis(moveY) * Math.abs(driverStick.getRawAxis(moveY)));
        double rotate =  driverStick.getRawAxis(rotateX) * -.3;

        if (drive.mag() < 0.05) {
            drive = new Vector2();
        }
        if (Math.abs(rotate) < 0.05) {
            rotate = 0;
        }
        
        SwerveManager.rotateAndDrive(rotate, drive);

    }

    public static void operatorInput() {}

}
