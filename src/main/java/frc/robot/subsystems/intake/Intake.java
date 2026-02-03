package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake {
    private static TalonFX intakeMotor;
    private static TalonFX intakeAngleMotor;
    private static IntakeState state;
    private static double intakeMotorSpeed;
    private static double intakeAngleMotorSpeed;
    /*
     * add comments to your class, 
     * make sure intakevisualzier does not look at state but simply angless. 
     * to that end set like angle varables of set angle and target speed as wwelll
     */

    public static void init(){
        intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
        intakeAngleMotor = new TalonFX(Constants.Intake.INTAKE_ANGLE_MOTOR_ID);
        state = IntakeState.RESTING;
        IntakeVisualizer.init();
    }

    public static void update(){
        // Switch states
        switch (state) {
            case RESTING:
                resting();
            break;
            
            case DOWN:
                down();
            break;
        
            case INTAKING:
                intake();
            break;
        }
        IntakeVisualizer.update();
    }

    public static void startRest() {
        state = IntakeState.RESTING;
    }

    public static void startDown() {
        state = IntakeState.DOWN;
    }

    public static void startIntaking() {
        state = IntakeState.INTAKING;
    }
     
    public static double getIntakeMotorSpeed(){
        return intakeMotorSpeed;
    }

    public static double getIntakeAngleMotorSpeed(){
        return intakeAngleMotorSpeed;
    }

    public static void resting(){
        intakeMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeMotorSpeed = 0;
        intakeAngleMotorSpeed = 0;
    }

    public static void down(){
        intakeAngleMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeMotorSpeed = 0;
        intakeAngleMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
    }

    public static void intake(){
        intakeMotor.set(Constants.Intake.MOTOR_INTAKE_SPEED);
        intakeMotorSpeed = Constants.Intake.MOTOR_INTAKE_SPEED;
        //
        intakeAngleMotorSpeed = 0;
    }
}