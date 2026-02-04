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
    
    /*
     * add comments to your class, 
     * make sure intakevisualzier does not look at state but simply angless. 
     * to that end set like angle varables of set angle and target speed as wwelll
     */

    public static void init(){
        intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_ID);
        intakeAngleMotor = new TalonFX(Constants.Intake.INTAKE_ANGLE_MOTOR_ID);
        state = IntakeState.RESTING;
        currentAngle = 0;
        IntakeVisualizer.init();
    }

    public static void update(){
        // Switch states
        switch (state) {
            case RESTING:
               resting();
               System.out.println("rest");
            break;
            
            case DOWN:
               down();
                System.out.println("down");
            break;
        
            case INTAKING:
               intake();
               System.out.println("intaking");
            break;

            case RETRACTING:
               retract();
               System.out.println("retracting");
            break;
        } 
         IntakeVisualizer.update();
        
    }

    public static void startRest() {
        state = IntakeState.RESTING;
    }

    public static void startDown() {
        state = IntakeState.DOWN;
        initialTime = Timer.getFPGATimestamp();
        currentAngle = 0;
    }

    public static void startRetract() {
        state = IntakeState.RETRACTING;
        initialTime = Timer.getFPGATimestamp();
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
      //  System.out.println("At rest");
        intakeMotor.set(Constants.Intake.MOTOR_REST_SPEED);
        intakeMotorSpeed = 0;
        intakeAngleMotorSpeed = 0;

        intakeMotorSpeed = 0;

    }

    public static void down(){

        intakeMotorSpeed = 0;

        double armAngle = -(Math.PI / 4);

        double timeTotal = 5;

        rotSpeed = armAngle / timeTotal;

        if(Timer.getFPGATimestamp() >= initialTime + timeTotal){
            System.out.println("Down stopped");
                intakeAngleMotor.set(0);
                intakeAngleMotorSpeed = 0;
        } else {
            System.out.println("Going down");
            intakeAngleMotor.set(rotSpeed);
            intakeAngleMotorSpeed = rotSpeed;
            currentAngle = rotSpeed * (Timer.getFPGATimestamp() - initialTime);
        }
        



    }

    public static void intake(){
        IntakeVisualizer.motorSpinning = true;
        System.out.println("Intaking");
        intakeMotor.set(Constants.Intake.MOTOR_INTAKE_SPEED);
        intakeMotorSpeed = Constants.Intake.MOTOR_INTAKE_SPEED;
    }

    public static void retract(){
        intakeMotorSpeed = 0;
        IntakeVisualizer.motorSpinning = false;

        double timeTotal = 5;

        System.out.println(rotSpeed);

        if(Timer.getFPGATimestamp() >= initialTime + timeTotal){
            System.out.println("Down stopped");
                intakeAngleMotor.set(0);
                intakeAngleMotorSpeed = 0;
        } else {
            System.out.println("Going up");
            intakeAngleMotor.set(rotSpeed);
            intakeAngleMotorSpeed = rotSpeed;
            currentAngle = (rotSpeed * (initialTime - Timer.getFPGATimestamp())) - (Math.PI) / 4;
        }
    }

}