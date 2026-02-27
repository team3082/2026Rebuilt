package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controllermaps.LogitechF310;
import frc.robot.subsystems.AutoTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.subsystems.states.ShooterTarget;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.Vector2;

public class OI {
    private static Joystick driverStick, operatorStick;

    // ------------------ Driver Controls ------------------ //

    // Movement
    private static final int moveX  = LogitechF310.AXIS_LEFT_X;
    private static final int moveY = LogitechF310.AXIS_LEFT_Y;
    private static final int rotateX  = LogitechF310.AXIS_RIGHT_X;
 
    // zero is for Pigeon 
    private static final int zero = LogitechF310.BUTTON_Y;

    private static final int toggleIntake = LogitechF310.BUTTON_LEFT_BUMPER;
    private static boolean intakeToggled = false;
    private static final int reverseIntake = LogitechF310.AXIS_LEFT_TRIGGER;

    private static final int shoot = LogitechF310.BUTTON_RIGHT_BUMPER;

    private static final int zeroTurret = LogitechF310.BUTTON_X;
    private static final int zeroHood = LogitechF310.BUTTON_A;

    private static final int intakeFeed = LogitechF310.AXIS_RIGHT_TRIGGER;

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

        // SCORING

        // intake
        if (driverStick.getRawButtonPressed(toggleIntake)) {
            intakeToggled = !intakeToggled; // intake toggles from on to off when button pressed
        }

        if (driverStick.getRawAxis(reverseIntake) > 0.25) {
            Intake.reverse();
            intakeToggled = false; // toggle goes off so it stops when reverse button is released
        } else if (intakeToggled) {
            Intake.startIntaking();
        } else if (driverStick.getRawAxis(intakeFeed) > 0.10) {
            System.out.println("it is");
            Intake.startFeeding(driverStick.getRawAxis(intakeFeed));
        } else {
            Intake.stopIntaking();
        }

        // shooter
        ShooterManager.setTarget(AutoTarget.getTarget());
        
        if (AutoTarget.nearTrench() && ShooterManager.getTarget() != ShooterTarget.HUB) { // prevents us from decapitation under the trench
            ShooterManager.stopShooting();
        } else {
            if (driverStick.getRawButton(shoot)) { // toggles shooting on and off when this button pressed
                ShooterManager.shoot();
            } else {
                ShooterManager.stopShooting();
            }
        }

        if (driverStick.getRawButton(zeroTurret)) {
            ShooterManager.zeroTurret();
        }

        if (driverStick.getRawButton(zeroHood)) {
            ShooterManager.zeroHood();
        }

    }

    private static void operatorInput() {}

}
