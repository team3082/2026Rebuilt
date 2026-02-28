package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.utils.RTime;

public class Turret {

    public enum TurretState {
        ZEROING,
        ZEROING_REVERSE, // if we hit the hardstop, assumes we started on wrong side of sensor and reverses direction
        NORMAL
    }

    private TalonFX turretMotor;
    private DigitalInput hallEffectSensor;
    
    private TurretState turretState = TurretState.ZEROING;
    private double targetAngle = 0.0;
    private double zeroStartTime = RTime.now(); // makes sure that current detection for hitting hardstop doesn't activate due to starting acceleration of turret

    /**
     * Initializion the turret
     */
    public Turret() {
        hallEffectSensor = new DigitalInput(Constants.Shooter.HALL_EFFECT_SENSOR_ID);

        turretMotor = new TalonFX(Constants.Shooter.TURRET_MOTOR_ID, "CANivore");
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
        turretMotorConfig.Slot0.kP = Tuning.Shooter.TURRET_KP;
        turretMotorConfig.Slot0.kI = Tuning.Shooter.TURRET_KI;
        turretMotorConfig.Slot0.kD = Tuning.Shooter.TURRET_KD;

        turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Tuning.Shooter.TURRET_VEL;
        turretMotorConfig.MotionMagic.MotionMagicAcceleration = Tuning.Shooter.TURRET_ACCEL;
        turretMotorConfig.MotionMagic.MotionMagicJerk = Tuning.Shooter.TURRET_JERK;

        turretMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        turretMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        turretMotor.getConfigurator().apply(turretMotorConfig);
        
    }

    /**
     * Updates the turret
     */
    public void update() {
        switch (turretState) {
            case ZEROING:
                if (Robot.isReal()){

                    if (Intake.getAngle() < Constants.Intake.INTAKE_SAFE_ANGLE) {
                        turretMotor.set(0);
                        return;
                    }

                    turretMotor.set(Tuning.Shooter.TURRET_ZEROING_SPEED);
                    
                    if (atZeroPosition()) {
                        turretMotor.setPosition(angleToRot(Constants.Shooter.TURRET_ZERO_ANGLE));
                        turretState = TurretState.NORMAL;
                    }

                    if (atHardstop()) {
                        System.out.println("hardstopping");
                        turretState = TurretState.ZEROING_REVERSE;
                        zeroStartTime = RTime.now();
                    }

                } else {
                    turretState = TurretState.NORMAL;
                }
                break;

            case ZEROING_REVERSE:
                if (Robot.isReal()){
                    turretMotor.set(-Tuning.Shooter.TURRET_ZEROING_SPEED);
                    
                    if (atZeroPosition()) {
                        turretMotor.setPosition(Constants.Shooter.TURRET_ZERO_ANGLE);
                        turretState = TurretState.NORMAL;
                    }

                    if (atHardstop()) {
                        turretMotor.setPosition(Constants.Shooter.TURRET_HARDSTOP_ZERO_ANGLE);
                        turretState = TurretState.NORMAL;
                    }
                } else {
                    turretState = TurretState.NORMAL;
                }
                break;
                
            case NORMAL:
                turretMotor.setControl(new MotionMagicDutyCycle(angleToRot(targetAngle)));
                break;
        }
    }

    /**
     * Sets the target angle for the turret.
     * @param angle Target angle in radians
     */
    public void setAngle(double angle) {
        targetAngle = Math.max(clampAngle(angle), Constants.Shooter.TURRET_MIN_ANGLE);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Clamps the angle to the range that turret can rotate to
     * @param angle
     * @return clamped angle within min and max turret angle, returns -10 if outside of turret rotation zone
     */
    public double clampAngle(double angle) {
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
    public double getAngle() {
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
    public boolean atAngle() {
        return Math.abs(getAngle() - targetAngle) < Tuning.Shooter.TURRET_DEADBAND && turretState == TurretState.NORMAL;
    }

    public TurretState getTurretState() {
        return turretState;
    }

    /**
     * Switches the turret to zeroing for reseting the encoder
     */
    public void zero(){
        turretState = TurretState.ZEROING;
        zeroStartTime = RTime.now();
    }

    /**
     * Checks if turret is against hardstop by sensing current draw
     * @return if turrent base is hitting hardstop
     */
    public boolean atHardstop() {
        return turretMotor.getStatorCurrent().getValueAsDouble() > 25.0 && turretMotor.getVelocity().getValueAsDouble() < 0.05 && RTime.now() - 0.15 > zeroStartTime;
    }

    public boolean atZeroPosition() {
        return !hallEffectSensor.get();
    }

    /**
     * returns the motor rotations at a given turret angle
     * @param radians turret angle in radians
     * @return
     */
    private double angleToRot(double radians) {
        return radians / 2.0 / Math.PI * Constants.Shooter.TURRET_GEAR_RATIO;
    }

    /**
     * returns the angle of the turret (relative to the robot) given motor rotations
     * @param rot
     * @return angle in radians
     */
    private double rotToAngle(double rot) {
        return rot * 2.0 * Math.PI / Constants.Shooter.TURRET_GEAR_RATIO;
    }

}