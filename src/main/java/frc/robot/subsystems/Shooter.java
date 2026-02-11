package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;

public class Shooter {
    private TalonFX flywheelMotor;
    private TalonFX hoodMotor;
    
    private double targetFlywheelSpeed = 0.0; // rotations per minute
    private double targetHoodAngle = 0.0; // radians

    /**
     * Initializes the flywheel motors and controllers.
     */
    public Shooter() {
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
    public void update() {
        hoodMotor.setControl(new PositionDutyCycle(hoodAngleToRot(targetHoodAngle)));

        switch (ShooterManager.getShooterState()) {
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
    public void setTargetAngle(double angle) {
        targetHoodAngle = angle;
    }

    public double getTargetAngle() {
        return targetHoodAngle;
    }

    /**
     * Sets the target flywheel speed.
     * @param speed Target speed in rotations per minute
     */
    public void setTargetSpeed(double speed) {
        targetFlywheelSpeed = speed / 60.0;
    }

    /**
     * Returns target flywheel speed
     * @return target flywheel speed in rotations per minute
     */
    public double getTargetSpeed() {
        return targetFlywheelSpeed * 60.0;
    }

    /**
     * Gets the current hood angle.
     * @return Current angle in radians
     */
    public double getAngle() {
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
    public double getVelocity() {
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
    public boolean atAngle() {
        return Math.abs(getAngle() - targetHoodAngle) < Tuning.Shooter.HOOD_DEADBAND;
    }

    /**
     * Checks if the flywheel is at the target speed.
     * @return if it is at target speed
     */
    public boolean atRampedSpeed() {
        return Math.abs(targetFlywheelSpeed - flywheelMotor.getVelocity().getValueAsDouble()) < Tuning.Shooter.FLYWHEEL_SPEED_DEADBAND;
    }

    /**
     * Converts hood angle to motor rotations
     * @param radians angle that hood that rotated
     * @return motor rotations for hood to be at given angle
     */
    private double hoodAngleToRot(double radians) {
        return radians / 2.0 / Math.PI * Constants.Shooter.HOOD_GEAR_RATIO;
    }

    /**
     * Converts motor rotations to hood angle
     * @param rot motor rotations
     * @return hood angle that will be at given motor rotations
     */
    private double rotToHoodAngle(double rot) {
        return rot * 2.0 * Math.PI / Constants.Shooter.HOOD_GEAR_RATIO;
    }

}