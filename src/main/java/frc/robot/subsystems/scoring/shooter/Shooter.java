package frc.robot.subsystems.scoring.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Tuning;

public class Shooter {
    public static TalonFX flywheelMotor;
    public static TalonFX hoodMotor;
    
    private static double targetFlywheelSpeed = 0.0; // rotations per second
    private static double targetHoodAngle = 0.0; // radians

    /**
     * Initializes the flywheel motors and controllers.
     */
    public static void init() {
        flywheelMotor = new TalonFX(Constants.Shooter.FLYWHEEL_MOTOR_ID);
        hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
        
        flywheelMotor.getConfigurator().apply(new TalonFXConfiguration());
        hoodMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.Slot0.kP = Tuning.Shooter.HOOD_KP;
        hoodConfiguration.Slot0.kI = Tuning.Shooter.HOOD_KI;
        hoodConfiguration.Slot0.kD = Tuning.Shooter.HOOD_KD;

        hoodMotor.getConfigurator().apply(hoodConfiguration);
        hoodMotor.setPosition(0);
        
    }

    /**
     * Updates the shooter
     */
    public static void update() {
        hoodMotor.setControl(new PositionDutyCycle(hoodAngleToRot(targetHoodAngle)));

        switch (ShooterManager.shooterState) {
            case IDLE:
                flywheelMotor.setControl(new CoastOut());
                break;
            
            default:
                flywheelMotor.setControl(new VelocityDutyCycle(targetFlywheelSpeed));
                break;
        }
    }

    /**
     * Sets the target hood angle.
     * @param angle Target angle in radians
     */
    public static void setTargetAngle(double angle) {
        targetHoodAngle = angle;
    }

    /**
     * Sets the target flywheel speed.
     * @param speed Target speed in rotations per second
     */
    public static void setTargetSpeed(double speed) {
        targetFlywheelSpeed = speed;
    }

    /**
     * Gets the current hood angle.
     * @return Current angle in radians
     */
    public static double getAngle() {
        return rotToHoodAngle(hoodMotor.getPosition().getValueAsDouble());
    }

    /**
     * Gets the current flywheel velocity.
     * @return Current velocity in rotations per second
     */
    public static double getVelocity() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Checks if the hood is at the target angle.
     * @return True if at target angle within tolerance
     */
    public static boolean atAngle() {
        return Math.abs(getAngle() - targetHoodAngle) < Tuning.Shooter.HOOD_DEADBAND;
    }

    /**
     * Checks if the flywheel is at the target speed.
     * @return if it is at target speed
     */
    public static boolean atRampedSpeed() {
        return Math.abs(targetFlywheelSpeed - flywheelMotor.getVelocity().getValueAsDouble()) < Tuning.Shooter.FLYWHEEL_SPEED_DEADBAND;
    }

    private static double hoodAngleToRot(double radians) {
        return radians / 2.0 / Math.PI * Constants.Shooter.HOOD_GEAR_RATIO;
    }

    private static double rotToHoodAngle(double rot) {
        return rot * 2.0 * Math.PI / Constants.Shooter.HOOD_GEAR_RATIO;
    }

}