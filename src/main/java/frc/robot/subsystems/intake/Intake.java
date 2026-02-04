package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Intake {
    private static TalonFX intakeMotor;
    private static TalonFX intakeAngleMotor;
    private static IntakeState state;
    private static double intakeMotorSpeed;
    private static double intakeAngleMotorSpeed;

    private static double initialTime;

    public static double currentAngle;

    public static double rotSpeed;
  
    public static void init(){
        intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
        intakeAngleMotor = new TalonFX(Constants.Intake.INTAKE_ANGLE_MOTOR_ID);
        state = IntakeState.RESTING;
        currentAngle = 0;
        IntakeVisualizer.init();
    }

    public static void update(){
        // states of the intake
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

            case RETRACTING:
               retract();
            break;
        } 
        IntakeVisualizer.update();
    }

    public static void startRest() {
        state = IntakeState.RESTING;
    }

    public static void startDown() {
        state = IntakeState.DOWN;
        // time in which sequence starts
        initialTime = Timer.getFPGATimestamp();
        // starts retracted
        currentAngle = 0;
    }

    public static void startRetract() {
        state = IntakeState.RETRACTING;
        // time in which sequence starts
        initialTime = Timer.getFPGATimestamp();
        // starts already extended
        currentAngle = -(Math.PI / 4);
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
        // stop all motors, in neutral position
        intakeMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeMotorSpeed = 0;
        intakeAngleMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeAngleMotorSpeed = 0;
    }

    public static void down(){
        // not intaking
        intakeMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
        intakeMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        // angle to travel
        double armAngle = -(Math.PI / 4);
        // time to travel
        double timeTotal = 0.3;

        rotSpeed = armAngle / timeTotal;

        // if timeTotal seconds have passed
        if(Timer.getFPGATimestamp() >= initialTime + timeTotal){
            intakeAngleMotor.set(Constants.Intake.MOTOR_REST_SPEED);
            intakeAngleMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
        } else {
            intakeAngleMotor.set(rotSpeed);
            intakeAngleMotorSpeed = rotSpeed;
            // changes angle gradually based on time and speed, useful for sim
            currentAngle = rotSpeed * (Timer.getFPGATimestamp() - initialTime);
        }
    }

    public static void intake(){
        IntakeVisualizer.motorSpinning = true;
        intakeMotor.set(Constants.Intake.MOTOR_INTAKE_SPEED);
        intakeMotorSpeed = Constants.Intake.MOTOR_INTAKE_SPEED;

        // ensure arm is not moving
        intakeAngleMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeAngleMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
    }

    public static void retract(){
        // not intaking
        intakeMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
        intakeMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        IntakeVisualizer.motorSpinning = false;
        // time to travel
        double timeTotal = 0.3;

        // if timeTotal seconds have passed
        if(Timer.getFPGATimestamp() >= initialTime + timeTotal){
            intakeAngleMotor.set(Constants.Intake.MOTOR_REST_SPEED);
            intakeAngleMotorSpeed = Constants.Intake.MOTOR_REST_SPEED;
        } else {
            intakeAngleMotor.set(rotSpeed);
            intakeAngleMotorSpeed = rotSpeed;
            // changes angle gradually to go in the direction to retract
            currentAngle = (rotSpeed * (initialTime - Timer.getFPGATimestamp())) - (Math.PI) / 4;
        }
    }

}