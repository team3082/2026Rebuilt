package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.states.IntakeState;
// import frc.robot.subsystems.visualizer.IntakeVisualizer;

public class Intake {
    private static TalonFX intakeMotor;
     private static IntakeState state;
     private static double speed;

    public static void init(){
        intakeMotor = new TalonFX(Constants.Intake.MOTOR_ID);
        state = IntakeState.RESTING;
        // IntakeVisualizer.init();
    }

    
    public static void update(){
        switch (state) {
            case RESTING:
                intakeMotor.set(0);
                speed = 0;
                break;
        
            case INTAKING:
                intakeMotor.set(Constants.Intake.SPEED);
                speed = Constants.Intake.SPEED;
                break;
        }

        // IntakeVisualizer.update();
    }

    public static void startIntaking() {
        state = IntakeState.INTAKING;
    }
    
    public static void stopIntaking() {
        state = IntakeState.RESTING;
    }
    public static double getSpeed(){
        return speed;
    }
}