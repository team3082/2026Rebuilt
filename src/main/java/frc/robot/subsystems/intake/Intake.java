package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class Intake {
    private static TalonFX intakeMotor;
    private static TalonFX downMotor;
    private static IntakeState state;
    private static double speed;
    private static double downSpeed;

    public static void init(){
        intakeMotor = new TalonFX(Constants.Intake.MOTORINTAKE_ID);
        downMotor = new TalonFX(Constants.Intake.MOTORDOWN_ID);
        state = IntakeState.RESTING;
        IntakeVisualizer.init();
    }

    public static void update(){
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

    public static void stopIntake() {
        state = IntakeState.RESTING;
    }

    public static void startDown() {
        state = IntakeState.DOWN;
    }

    public static void startIntaking() {
        state = IntakeState.INTAKING;
    }
     
    public static double getSpeed(){
        return speed;
    }

    public static double getDownSpeed(){
        return downSpeed;
    }

    public static void resting(){
        intakeMotor.set(Constants.Intake.MOTORRESTSPEED);
        speed = 0;
        downSpeed = 0;
    }

    public static void down(){
        downMotor.set(Constants.Intake.MOTORRESTSPEED);
        speed = 0;
        downSpeed = Constants.Intake.MOTORRESTSPEED;
    }

    public static void intake(){
        intakeMotor.set(Constants.Intake.MOTORINTAKESPEED);
        speed = Constants.Intake.MOTORINTAKESPEED;
        downSpeed = 0;
    }
}