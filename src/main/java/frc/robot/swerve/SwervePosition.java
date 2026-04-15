package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.vision.VisionManager;

public class SwervePosition {

    // Smoothly correct field position based on vision output. VISION_CORRECTION_FACTOR should range from 0.0 to
    // 1.0, representing the speed at which we blend from the odometry output to the output of the vision. 
    //static final double VISION_CORRECTION_FACTOR = 0.1;

    private static Vector2 position;
    private static Vector2 absVelocity;
    private static Vector2 lastOdomPos;
    private static Vector2 difference;


    // for kalman filtering, may need tuning
    private static Matrix<N2, N1> prediction = VecBuilder.fill(0, 0);
    // 2x2 matrices
    private static Matrix<N2, N2> uncertainty = new Matrix<>(Nat.N2(), Nat.N2());;
    private static final Matrix<N2, N2> odometryError =  new Matrix<>(Nat.N2(), Nat.N2()); 
    private static final Matrix<N2, N2> visionError = new Matrix<>(Nat.N2(), Nat.N2());   


    public static void init() {
        absVelocity = new Vector2(0.0,0.0);
        position = new Vector2(0.0,0.0);
        lastOdomPos = new Vector2(0.0,0.0);
        Odometry.init();

        // for kalman filtering, these values may need tuning
        uncertainty.set(0,0,0.01);
        uncertainty.set(1,1,0.01);
        odometryError.set(0,0,0.01);
        odometryError.set(1,1,0.01);
        visionError.set(0,0,0.01);
        visionError.set(1,1,0.01);
    }

    public static void update() {
        Vector2 odometryPos = Odometry.getPosition();
        Vector2 odometryInnovation = odometryPos.sub(lastOdomPos);
        
        position = new Vector2(prediction.get(0, 0), prediction.get(1, 0));
        difference = odometryPos.sub(lastOdomPos);
        predict();
        lastOdomPos = odometryPos;

        

        absVelocity = odometryInnovation.div(RTime.deltaTime());

        //System.out.println("lala odometry: " + odometryPos);
    }

    public static void predict(){
        // predicting next position based on previous position and current velocity
        prediction = prediction.plus(VecBuilder.fill(difference.x, difference.y));
        
        // wheels may shift
        uncertainty.plus(odometryError);

        Matrix<N2, N2> kalmanGain = uncertainty.times((uncertainty.plus(visionError)).inv());

        // recalculate position and uncertainty
        position = prediction.plus(kalmanGain.times(VisionManager.getPosition()));
        
    }

    //public static final double correctionMultiplier = 0.1;

    /**
     * Returns array of the robot's angle and distance in INCHES based of manual calculations
     */
    public static double[] getPositionPolar() {
        
        Vector2 pos = getPosition();
        double distance = pos.mag();
        double angleRad = pos.atan2();

        return new double[]{ angleRad, distance };
    }

    public static Vector2 getPosition() {
        return position;
    }

    public static Vector2 getAbsVelocity() {
        return absVelocity;
    }

    /**
     * Recalibrates the SwervePosition based on a position on the field. Should only be used when vision is disabled,
     * otherwise it'll just be overwritten the next frame.
     * @param newPosition the new position to set the robot position to
     */
    public static void setPosition(Vector2 newPosition) {
        Odometry.setPosition(newPosition);
        position = newPosition;
    }
    
    public static double getAngleOffsetToTarget(Vector2 desiredPosition){
        Vector2 currentPos = getPosition();
        Vector2 dif = new Vector2(desiredPosition.y - currentPos.y, desiredPosition.x - currentPos.x);
        return Math.PI/2 - dif.atan2();
    }

    /**
     * Return the current pose of the robot, adjusted for the rotation.
     */
    public static Pose2d getPose() {
        return new Pose2d(new Translation2d(position.x, position.y), Rotation2d.fromRadians(Pigeon.getRotationRad()));
    }

    
}



