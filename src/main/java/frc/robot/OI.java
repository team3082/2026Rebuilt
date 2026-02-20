package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controllermaps.LogitechF310;
import frc.robot.subsystems.AutoTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterManager;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.Vector2;

public class OI {
    private static Joystick driverStick;

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

    private static final int toggleShooter = LogitechF310.BUTTON_RIGHT_BUMPER;
    private static boolean shooterToggled = false;
    private static final int activateShooter = LogitechF310.AXIS_RIGHT_TRIGGER;

    private static final int zeroTurret = LogitechF310.BUTTON_X;

    /**
     * Initialize OI with preset joystick ports.
     */
    public static void init() {
        driverStick = new Joystick(0);
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
        } else {
            Intake.stopIntaking();
        }

        // shooter
        ShooterManager.setTarget(AutoTarget.getTarget());
        
        if (AutoTarget.nearTrench()) { // prevents us from decapitation under the trench
            ShooterManager.stopShooting();
        } else {
            if (driverStick.getRawButtonPressed(toggleShooter)) { // toggles shooting on and off when this button pressed
                shooterToggled = !shooterToggled;
            }

            if (driverStick.getRawAxis(activateShooter) > 0.25) { // shoots while this button is pressed for option that is not a toggle, may remove later because it is redundant
                ShooterManager.shoot();
                shooterToggled = false; // toggles shooter off if we shoot with this button
            } else {
                if (shooterToggled) {
                    ShooterManager.shoot();
                } else {
                    ShooterManager.stopShooting();
                }
            }
        }

        if (driverStick.getRawButton(zeroTurret)) {
            ShooterManager.getTurret().zero();
        }

    }

    private static void operatorInput() {}

}
