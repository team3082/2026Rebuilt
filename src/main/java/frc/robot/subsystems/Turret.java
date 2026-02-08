package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;

public class Turret {

    public enum TurretState {
        ZEROING,
        NORMAL
    }

    private static TalonFX turretMotor;
    private static DigitalInput hallEffectSensor;
    
    private static TurretState turretState = TurretState.ZEROING;
    private static double targetAngle = 0.0;

    /**
     * Initializion the turret
     */
    public static void init() {
        hallEffectSensor = new DigitalInput(Constants.Shooter.HALL_EFFECT_SENSOR_ID);

        turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID);
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.Slot0.kP = Tuning.Shooter.TURRET_KP;
        turretMotorConfig.Slot0.kI = Tuning.Shooter.TURRET_KI;
        turretMotorConfig.Slot0.kD = Tuning.Shooter.TURRET_KD;

        turretMotor.getConfigurator().apply(turretMotorConfig);
        
    }

    /**
     * Updates the turret
     */
    public static void update() {
        switch (turretState) {
            case ZEROING:
                if (Robot.isReal()){
                    turretMotor.set(Tuning.Shooter.TURRET_ZEROING_SPEED);
                    
                    if (atZeroPosition()) {
                        turretMotor.setPosition(Constants.Shooter.TURRET_ZERO_ANGLE);
                        turretState = TurretState.NORMAL;
                    }
                } else {
                    turretState = TurretState.NORMAL;
                }
                break;
                
            case NORMAL:
                turretMotor.setControl(new PositionDutyCycle(angleToRot(targetAngle)));
                break;
        }
    }

    /**
     * Sets the target angle for the turret.
     * @param angle Target angle in radians
     */
    public static void setAngle(double angle) {
        targetAngle = Math.max(clampAngle(angle), Constants.Shooter.TURRET_MIN_ANGLE);
    }

    public static double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Clamps the angle to the range that turret can rotate to
     * @param angle
     * @return clamped angle, returns -10 if outside of turret rotation zone
     */
    public static double clampAngle(double angle) {
        while (angle > Constants.Shooter.TURRET_MAX_ANGLE) {
            angle -= 2.0 * Math.PI;
        }

        while (angle <= Constants.Shooter.TURRET_MAX_ANGLE - 2.0 * Math.PI) {
            angle += 2.0 * Math.PI;
        }

        if (angle < Constants.Shooter.TURRET_MIN_ANGLE) return -10;
        return angle;
    }

    /**
     * Gets the current angle of the turret.
     * @return Current angle in radians
     */
    public static double getAngle() {
        if (Robot.isReal()){
            return rotToAngle(turretMotor.getPosition().getValueAsDouble());
        } else {
            return targetAngle;
        }
    }

    /**
     * Checks if the turret is at the target angle.
     * @return True if at target angle within deadband
     */
    public static boolean atAngle() {
        return Math.abs(getAngle() - targetAngle) < Tuning.Shooter.TURRET_DEADBAND && turretState == TurretState.NORMAL;
    }

    public static TurretState getTurretState() {
        return turretState;
    }

    /**
     * Switches the turret to zeroing for reseting the encoder
     */
    public static void zero(){
        turretState = TurretState.ZEROING;
    }

    public static boolean atZeroPosition() {
        return !hallEffectSensor.get();
    }

    /**
     * returns the motor rotations at a given turret angle
     * @param radians turret angle in radians
     * @return
     */
    private static double angleToRot(double radians) {
        return radians / 2.0 / Math.PI * Constants.Shooter.TURRET_GEAR_RATIO;
    }

    /**
     * returns the angle of the turret (relative to the robot) given motor rotations
     * @param rot
     * @return angle in radians
     */
    private static double rotToAngle(double rot) {
        return rot * 2.0 * Math.PI / Constants.Shooter.TURRET_GEAR_RATIO;
    }

}