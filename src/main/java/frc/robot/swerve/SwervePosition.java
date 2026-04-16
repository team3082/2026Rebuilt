package frc.robot.swerve;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.vision.VisionManager;

/**
 * Manages the robot's field-relative position using a Kalman Filter.
 * This class fuses Odometry data (high frequency, prone to drift) with 
 * Vision data (lower frequency, high absolute accuracy) to provide a stable Pose.
 */
public class SwervePosition {

    // Final output states
    private static Vector2 position = new Vector2(0, 0);
    private static Vector2 absVelocity = new Vector2(0, 0);
    private static Vector2 lastOdomPos = new Vector2(0, 0);

    /*
     * The State Estimate matrix [x, y]. 
     * This is the "internal" version of our position used for math.
     */
    private static Matrix<N2, N1> stateEstimate = VecBuilder.fill(0, 0);

    /** 
     * The Covariance Matrix (P). Represents our confidence in the current position.
     * Higher values mean we are less certain where we are.
     */
    private static Matrix<N2, N2> uncertainty = Matrix.eye(Nat.N2()).times(0.1);
    
    /**
     * Base Process Noise (Q). This is how much we "trust" odometry per meter traveled.
     */
    private static final double ODOM_TRUST_COEFFICIENT = 0.05; 

    /** 
     * Sensor Noise (R). This is how much we trust the vision system.
     * A value of 0.01 means we trust vision a lot; 0.5 means vision is "noisy."
     */
    private static final Matrix<N2, N2> R_VISION = Matrix.eye(Nat.N2()).times(0.01);   
    
    /**
     * Node map with stored positions with correspoding time values.
     */
    private static TreeMap<Matrix<N2, N1>, Double> poseHistory = new TreeMap<>();

    public static void init() {
        position = new Vector2(0, 0);
        absVelocity = new Vector2(0, 0);
        lastOdomPos = new Vector2(0, 0);
        stateEstimate = VecBuilder.fill(0, 0);
        
        uncertainty = Matrix.eye(Nat.N2()).times(0.1);
        
        Odometry.init();
    }

    /**
     * The main loop for position tracking. 
     * Should be called in a periodic method (e.g., Robot.robotPeriodic).
     */
    public static void update() {
        //Get Change
        Vector2 currentOdomPos = Odometry.getPosition();
        Vector2 odomDelta = currentOdomPos.sub(lastOdomPos);
        double distanceTraveled = odomDelta.mag();

        //Predict new position based on odometry
        predict(odomDelta, distanceTraveled);
        
        //remove all elements from .5 seconds ago
        while (!poseHistory.isEmpty() && RTime.now() - poseHistory.firstEntry().getValue() > 0.5) {
            poseHistory.pollFirstEntry();
        }

        Optional<Matrix<N2, N1>> visionMeasurement = VisionManager.getMatrixPosition();
        if (visionMeasurement.isPresent()) {
            correct(visionMeasurement.get());
        }

        //Save to position buffer
        poseHistory.put(stateEstimate, VisionManager.getTimestampSeconds());

        //Corrects stateEstimate based on latency
        Double timeDifference = VisionManager.getTimestampSeconds() - VisionManager.getLatency();
        Matrix<N2, N1> error = VisionManager.getMatrixPosition().get().minus(poseHistory.get(timeDifference));
        stateEstimate.minus(error);

        // Update the public position and velocity based on the internal state estimate
        position = new Vector2(stateEstimate.get(0, 0), stateEstimate.get(1, 0));
        
        double dt = RTime.deltaTime();
        absVelocity = (dt > 0) ? odomDelta.div(dt) : new Vector2(0, 0);
        
        lastOdomPos = currentOdomPos;
    }

    /**
     * Prediction Step:
     * We add the delta from odometry to our current estimate.
     * We also increase uncertainty based on how far we moved.
     */
    private static void predict(Vector2 delta, double distance) {
        // Move the estimate
        stateEstimate = stateEstimate.plus(VecBuilder.fill(delta.x, delta.y));

        // Grow uncertainty dynamically: P = P + (distance * Q_coeff)
        // This ensures that if we are sitting still, the uncertainty doesn't explode.
        Matrix<N2, N2> dynamicProcessNoise = Matrix.eye(Nat.N2()).times(distance * ODOM_TRUST_COEFFICIENT);
        uncertainty = uncertainty.plus(dynamicProcessNoise);
    }

    /**
     * Correction Step (The "Kalman" part):
     * Merges the vision data with our predicted state.
     */
    private static void correct(Matrix<N2, N1> measurement) {
        // Calculate Kalman Gain (K) ue How much do we trust vision vs. our prediction?
        // K = uncertainty / (uncertainty + vision_noise)
        Matrix<N2, N2> kalmanGain = uncertainty.times((uncertainty.plus(R_VISION)).inv());
        
        // Calculate the difference between vision and prediction (Innovation)
        Matrix<N2, N1> innovation = measurement.minus(stateEstimate);
        
        // Adjust the estimate based on the gain: x = x + K * innovation
        stateEstimate = stateEstimate.plus(kalmanGain.times(innovation));

        // Update the uncertainty: Because we have a new measurement, we are now MORE certain.
        // P = (I - K) * P
        uncertainty = Matrix.eye(Nat.N2()).minus(kalmanGain).times(uncertainty);
    }

    public static Vector2 getPosition() {
        return position;
    }

    public static Vector2 getAbsVelocity() {
        return absVelocity;
    }

    public static void setPosition(Vector2 newPosition) {
        Odometry.setPosition(newPosition);
        stateEstimate = VecBuilder.fill(newPosition.x, newPosition.y);
        position = newPosition;
        lastOdomPos = newPosition;

        // Reset uncertainty because we've been told exactly where we are
        uncertainty = Matrix.eye(Nat.N2()).times(0.01);
    }

    public static Pose2d getPose() {
        return new Pose2d(
            new Translation2d(position.x, position.y), 
            Rotation2d.fromRadians(Pigeon.getRotationRad())
        );
    }
}
