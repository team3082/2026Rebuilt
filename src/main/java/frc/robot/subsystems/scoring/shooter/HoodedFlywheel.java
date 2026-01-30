package frc.robot.subsystems.scoring.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class HoodedFlywheel {
    private static TalonFX flywheelMotor;
    private static TalonFX angleMotor;
    private static PIDController angleMotorPID;
    
    private static FlywheelState flywheelState = FlywheelState.IDLE;
    private static double targetedSpeed = 0.0;
    private static double targetedAngle = 0.0;
    private static double currentAngle = 0.0;
    private static double pastAngle = 0.0;

    /**
     * Initializes the flywheel motors and controllers.
     */
    public static void init() {
        flywheelMotor = new TalonFX(Constants.Shooter.FLYWHEEL_MOTOR_ID);
        angleMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID);
        
        angleMotorPID = new PIDController(
            Constants.Shooter.HOOD_KP,
            Constants.Shooter.HOOD_KI,
            Constants.Shooter.HOOD_KD
        );
        
        currentAngle = 0.0;
        pastAngle = angleMotor.getPosition().getValueAsDouble();
    }
    /**
     * Updates the Hooded Flywheel 
     */
    public static void update() {
        switch (flywheelState) {
            case IDLE:
                updateAngleMotor();
                flywheelMotor.set(0);
                break;
            case REVVING:
                updateAngleMotor();
                flywheelMotor.set(targetedSpeed);
                
                if (atAngle() && atRampedSpeed()) {
                    flywheelState = FlywheelState.SHOOTING;
                }
                
                break;
            case SHOOTING:
                updateAngleMotor();
                flywheelMotor.set(targetedSpeed);
                break;
            default:
                break;
        }
    }

    /**
     * Updates the angle encoder and the motor output
     */
    private static void updateAngleMotor(){
        updateAngle();

        double anglePidOutput = angleMotorPID.calculate(currentAngle, targetedAngle);
        angleMotor.set(anglePidOutput);
    }

    /**
     * Sets the target hood angle.
     * @param angle Target angle in degrees
     */
    public static void setAngle(double angle) {
        targetedAngle = angle;
    }

    /**
     * Sets the target flywheel speed.
     * @param speed Target speed (0.0 to 1.0)
     */
    public static void setSpeed(double speed) {
        targetedSpeed = speed;
    }

    /**
     * Sets the flywheel state.
     * @param state Desired flywheel state
     */
    public static void setState(FlywheelState state) {
        flywheelState = state;
    }

    /**
     * Gets the current hood angle.
     * @return Current angle in degrees
     */
    public static double getAngle() {
        return currentAngle;
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
        return Math.abs(currentAngle - targetedAngle) < Constants.Shooter.HOOD_ANGLE_TOLERANCE;
    }

    /**
     * Checks if the flywheel is at the target speed.
     * @return True if at target speed within tolerance
     */
    public static boolean atRampedSpeed() {
        double currentVelocity = flywheelMotor.getVelocity().getValueAsDouble();

        return Math.abs(targetedSpeed - currentVelocity) < Constants.Shooter.VELOCITY_TOLERANCE;
    }

    /**
     * Updates the current hood angle based on motor encoder position.
     */
    private static void updateAngle() {
        double currentPos = angleMotor.getPosition().getValueAsDouble();
        double deltaPos = currentPos - pastAngle;
        currentAngle += deltaPos * Constants.Shooter.HOOD_RATIO;
        pastAngle = currentPos;
    }
}