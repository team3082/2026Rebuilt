package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;

public class Intake {

    public enum IntakeState {
        RESTING,
        INTAKING,
        REVERSE,
        FEEDING,
    }

    private static TalonFX pivotMotor;
    private static TalonFX rollerMotor;
    public static IntakeState rollerState;

    private static double feedPercent; // percent that the intake should raise

    public static void init(){
        rollerState = IntakeState.INTAKING;

        rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
        pivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);

        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

        pivotConfig.Slot0.kP = Tuning.Intake.PIVOT_P;
        pivotConfig.Slot0.kI = Tuning.Intake.PIVOT_I;
        pivotConfig.Slot0.kD = Tuning.Intake.PIVOT_D;
        
        pivotConfig.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivotConfig);

    }

    
    public static void update(){
        switch (rollerState) {
            case RESTING:
                pivotMotor.setControl(new PositionDutyCycle(Constants.Intake.INTAKE_DOWN_ANGLE));
                rollerMotor.set(0);
                break;
        
            case INTAKING:
                pivotMotor.setControl(new PositionDutyCycle(Constants.Intake.INTAKE_DOWN_ANGLE));
                rollerMotor.set(Tuning.Intake.SPEED);
                break;

            case REVERSE:
                pivotMotor.setControl(new PositionDutyCycle(Constants.Intake.INTAKE_DOWN_ANGLE));
                rollerMotor.set(Tuning.Intake.REVERSE_SPEED);
                break;

            case FEEDING:
                double targetAngle = feedPercent * (Constants.Intake.INTAKE_UP_ANGLE - Constants.Intake.INTAKE_DOWN_ANGLE) + Constants.Intake.INTAKE_DOWN_ANGLE; // lets driver control how far intake raises

                pivotMotor.setControl(new PositionDutyCycle(targetAngle));
                rollerMotor.set(0);
                break;
        }
    }

    public static IntakeState getIntakeState() {
        return rollerState;
    }

    public static void startIntaking() {
        rollerState = IntakeState.INTAKING;
    }

    public static void reverse() {
        rollerState = IntakeState.REVERSE;
    }
    
    public static void stopIntaking() {
        rollerState = IntakeState.RESTING;
    }

    /**
     * Starts feeding by raising pivot angle
     * @param percent percent that the intake should raise up
     */
    public static void startFeeding(double percent) {
        rollerState = IntakeState.FEEDING;
        feedPercent = percent;
    }

    public static double getAngle() {
        if (Robot.isReal()) {
            return pivotMotor.getPosition().getValueAsDouble();
        } else {
            if (rollerState == IntakeState.FEEDING) {
                return feedPercent * (Constants.Intake.INTAKE_UP_ANGLE - Constants.Intake.INTAKE_DOWN_ANGLE) + Constants.Intake.INTAKE_DOWN_ANGLE;
            } else {
                return Constants.Intake.INTAKE_DOWN_ANGLE;
            }
        }
    }

    public static double getSpeed() {
        if (Robot.isReal()) {
            return rollerMotor.get();
        } else {
            switch (rollerState) {
                case RESTING:
                    return 0;
            
                case INTAKING:
                    return Tuning.Intake.SPEED;

                case REVERSE:
                    return Tuning.Intake.REVERSE_SPEED;
                
                case FEEDING:
                    return 0;                    
            }
        }
        return 0;
    }
}