package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.subsystems.LEDs.LEDManager;
import frc.robot.subsystems.LEDs.LEDManager.Colors;

public class Intake {

    public enum IntakeState {
        RESTING,
        INTAKING,
        REVERSE,
    }

    private static TalonFX pivotMotor;
    private static TalonFX rollerMotor;
    public static IntakeState rollerState;

    public static void init(){
        rollerState = IntakeState.INTAKING;

        rollerMotor = new TalonFX(Constants.Intake.ROLLER_MOTOR_ID);
        pivotMotor = new TalonFX(Constants.Intake.PIVOT_MOTOR_ID);

        rollerMotor.getConfigurator().apply(new TalonFXConfiguration());
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = Tuning.Intake.PIVOT_P;
        pivotConfig.Slot0.kI = Tuning.Intake.PIVOT_I;
        pivotConfig.Slot0.kD = Tuning.Intake.PIVOT_D;
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
}