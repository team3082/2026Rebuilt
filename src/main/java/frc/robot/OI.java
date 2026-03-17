package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.controllermaps.LogitechF310;
import frc.robot.subsystems.AutoTarget;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;

public class OI {
    private static Joystick driverStick, operatorStick;

    // ------------------ Driver Controls ------------------ //

    // Movement
    private static final int moveX  = LogitechF310.AXIS_LEFT_X;
    private static final int moveY = LogitechF310.AXIS_LEFT_Y;
    private static final int rotateX  = LogitechF310.AXIS_RIGHT_X;


    private static final int shoot = LogitechF310.BUTTON_RIGHT_BUMPER;
    private static final int zeroHood = LogitechF310.BUTTON_A;

 
    // zero is for Pigeon 
    private static final int zero = LogitechF310.BUTTON_Y;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
        operatorStick = new Joystick(1);
    }

    public static void userInput() {
        driverInput();
        operatorInput();
    }

    /**
     * Instruct the robot to follow instructions from joysticks.
     * One call from this equals one frame of robot instruction.
     * Because we used TimedRobot, this runs 50 times a second,
     * so this lives in the teleopPeriodic() function.
     */
    private static void driverInput() {
        // INPUT

        // Reset pigeon
        if (driverStick.getRawButton(zero)) Pigeon.reset();

        if (driverStick.getRawButtonPressed(zeroHood)) {
            ShooterManager.zeroHood();
        }

        if (driverStick.getRawButton(shoot)) {
            System.out.println("Shoot button pressed");
            double rawMoveX = driverStick.getRawAxis(moveX) * Math.abs(driverStick.getRawAxis(moveX));
            double rawMoveY = driverStick.getRawAxis(moveY) * Math.abs(driverStick.getRawAxis(moveY));
            Vector2 drive = new Vector2(rawMoveX, -rawMoveY);
            
            if (drive.mag() > 0.05) {
                System.out.println("Shoot and move");
                drive = drive.mul(0.3);

                Vector2 targetPos = ShooterManager.getTarget().pos;
                double targetAngle = Math.atan2(targetPos.y - SwervePosition.getPosition().y, targetPos.x - SwervePosition.getPosition().x);
                SwervePID.setDestState(SwervePosition.getPosition(), targetAngle);
                SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), drive);
                ShooterManager.shoot();

            } else {
                //Planting
                System.out.println("Shoot and plant");
                
           
                Vector2 targetPos = ShooterManager.getTarget().pos;
                double targetAngle = Math.atan2(targetPos.y - SwervePosition.getPosition().y, targetPos.x - SwervePosition.getPosition().x);
                SwervePID.setDestState(SwervePosition.getPosition(), targetAngle);
                double angleError = Math.atan2(Math.sin(targetAngle - Pigeon.getRotationRad()), Math.cos(targetAngle - Pigeon.getRotationRad()));

                if (Math.abs(angleError) < 0.2) {
                    System.out.println("At target angle, planting");
                    SwerveManager.plant();
                } else {
                    SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), new Vector2());
                }
                
                ShooterManager.shoot();

            }

        } else {
            ShooterManager.stopShooting();
            normalDrive();
        }
       
    }

    private static void normalDrive() {
        double rawMoveX = driverStick.getRawAxis(moveX) * Math.abs(driverStick.getRawAxis(moveX));
        double rawMoveY = driverStick.getRawAxis(moveY) * Math.abs(driverStick.getRawAxis(moveY));
        Vector2 drive = new Vector2(rawMoveX, -rawMoveY);
        
        double rotate =  driverStick.getRawAxis(rotateX) * -.3;
        
        if (drive.mag() < 0.05) {
            drive = new Vector2();
        }

        if (Math.abs(rotate) < 0.05) {
            rotate = 0;
        }

        SwerveManager.rotateAndDrive(rotate, drive);
    }

    private static void operatorInput() {}

}
