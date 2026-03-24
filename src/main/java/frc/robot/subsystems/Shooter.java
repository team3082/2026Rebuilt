package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;

public class Shooter {
    private static TalonFX flywheelMotor;
    private static TalonFX hoodMotor;
    
    private static double targetFlywheelSpeed = 0.0;
    private static double targetHoodAngle = 0.0;

    /**
     * Initializes the flywheel motors and controllers.
     */
    public static void init() {
        flywheelMotor = new TalonFX(Constants.Shooter.FLYWHEEL_MOTOR_ID, "CANivore");
        hoodMotor = new TalonFX(Constants.Shooter.HOOD_MOTOR_ID, "CANivore");
        
        flywheelMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
        flywheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        flywheelConfiguration.Slot0.kP = Tuning.Shooter.FLYWHEEL_P;
        flywheelConfiguration.Slot0.kI = Tuning.Shooter.FLYWHEEL_I;
        flywheelConfiguration.Slot0.kD = Tuning.Shooter.FLYWHEEL_D;
        flywheelConfiguration.Slot0.kV = Tuning.Shooter.FLYWHEEL_KV;

        flywheelConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        flywheelMotor.getConfigurator().apply(flywheelConfiguration);

        hoodMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.Slot0.kP = Tuning.Shooter.HOOD_KP;
        hoodConfiguration.Slot0.kI = Tuning.Shooter.HOOD_KI;
        hoodConfiguration.Slot0.kD = Tuning.Shooter.HOOD_KD;

        hoodConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        hoodConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodConfiguration);
        hoodMotor.setPosition(0);
        
    }

    /**
     * Updates the shooter
     */
    public static void update() {
        if (AutoTarget.nearTrench()) { // final safety check to override other potential errors
            targetHoodAngle = 0;
        }

        switch (ShooterManager.getShooterState()) {
            case IDLE:
                hoodMotor.setControl(new PositionDutyCycle(hoodAngleToRot(0)));
                flywheelMotor.setControl(new CoastOut());
                break;

            case ZEROING:
                flywheelMotor.setControl(new CoastOut());
                if (Robot.isReal()){
                    hoodMotor.set(Tuning.Shooter.HOOD_ZEROING_SPEED);
                    if (hoodMotor.getStatorCurrent().getValueAsDouble() > 80) {
                        hoodMotor.setPosition(0);  
                        ShooterManager.stopShooting();
                    }

                } else {
                    ShooterManager.stopShooting();
                }
                break;

            default:
                hoodMotor.setControl(new PositionDutyCycle(hoodAngleToRot(targetHoodAngle)));
                flywheelMotor.setControl(new VelocityVoltage(targetFlywheelSpeed));
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

    public static double getTargetAngle() {
        return targetHoodAngle;
    }

    /**
     * Sets the target flywheel speed.
     * @param speed Target speed in rotations per minute
     */
    public static void setTargetSpeed(double speed) {
        targetFlywheelSpeed = speed / 60.0;
    }

    /**
     * Returns target flywheel speed
     * @return target flywheel speed in rotations per minute
     */
    public static double getTargetSpeed() {
        return targetFlywheelSpeed * 60.0;
    }

    /**
     * Gets the current hood angle.
     * @return Current angle in radians
     */
    public static double getAngle() {
        if (Robot.isReal()) {
            return rotToHoodAngle(hoodMotor.getPosition().getValueAsDouble());
        } else {
            return targetHoodAngle;
        }
    }

    /**
     * Gets the current flywheel velocity.
     * @return Current velocity in rotations per minute
     */
    public static double getVelocity() {
        if (Robot.isReal()) {
            return flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
        } else {
            return targetFlywheelSpeed * 60.0;
        }
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
        return Math.abs(targetFlywheelSpeed - flywheelMotor.getVelocity().getValueAsDouble()) < (Tuning.Shooter.FLYWHEEL_SPEED_DEADBAND / 60.0);
    }

    /**
     * Converts hood angle to motor rotations
     * @param radians angle that hood that rotated
     * @return motor rotations for hood to be at given angle
     */
    private static double hoodAngleToRot(double radians) {
        return radians / 2.0 / Math.PI * Constants.Shooter.HOOD_GEAR_RATIO;
    }

    /**
     * Converts motor rotations to hood angle
     * @param rot motor rotations
     * @return hood angle that will be at given motor rotations
     */
    private static double rotToHoodAngle(double rot) {
        return rot * 2.0 * Math.PI / Constants.Shooter.HOOD_GEAR_RATIO;
    }

}