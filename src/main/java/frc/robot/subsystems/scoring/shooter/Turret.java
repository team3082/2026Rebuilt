package frc.robot.subsystems.scoring.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Turret {
    private static TalonFX motor;
    private static DigitalInput hallEffectSensor;
    private static PIDController turretPID;
    
    private static TurretState turretState = TurretState.ZEROING;
    private static double setAngle = 0.0;
    private static double currentAngle = 0.0;
    private static double pastAngle = 0.0;

    /**
     * Initializion the turret
     */
    public static void init() {
        hallEffectSensor = new DigitalInput(Constants.Shooter.HALL_EFFECT_SENSOR);
        turretPID = new PIDController(
            Constants.Shooter.TURRET_KP, 
            Constants.Shooter.TURRET_KI, 
            Constants.Shooter.TURRET_KD
        );    
        motor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);
        
        currentAngle = 0.0;
        pastAngle = motor.getPosition().getValueAsDouble();
    }

    /**
     * Updates the turret
     */
    public static void update() {
        switch (turretState) {
            case ZEROING:
                motor.set(Constants.Shooter.ZEROING_SPEED);
                
                if (hallEffectSensor.get()) {
                    motor.set(0);
                    currentAngle = 0.0;
                    pastAngle = motor.getPosition().getValueAsDouble();
                    turretState = TurretState.NORMAL;
                }
                break;
                
            case NORMAL:
                updateAngle();
                double pidOutput = turretPID.calculate(currentAngle, setAngle);
                motor.set(pidOutput);
                break;
                
            default:
                motor.set(0);
                break;
        }
    }

    /**
     * Sets the target angle for the turret.
     * @param angle Target angle in degrees
     */
    public static void setAngle(double angle) {
        setAngle = angle;
    }

    /**
     * Gets the current angle of the turret.
     * @return Current angle in degrees
     */
    public static double getAngle() {
        return currentAngle;
    }

    /**
     * Checks if the turret is at the target angle.
     * @return True if at target angle within deadband
     */
    public static boolean atAngle() {
        return Math.abs(currentAngle - setAngle) < Constants.Shooter.TURRET_DEADBAND;
    }

    /**
     * Switches the turret to zeroing for reseting the encoder
     */
    public static void zero(){
        turretState = TurretState.ZEROING;
    } 

    /**
     * Updates the current angle based on motor encoder position.
     */
    private static void updateAngle() {
        double currentPos = motor.getPosition().getValueAsDouble();
        double deltaPos = currentPos - pastAngle;
        currentAngle += deltaPos * Constants.Shooter.TURRET_RATIO;
        pastAngle = currentPos;
    }

}