package frc.robot.swerve;

import java.util.Map;
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
 * Fuses odometry (high frequency, prone to drift) with vision (lower frequency,
 * high absolute accuracy) via retroactive correction: when a vision measurement
 * arrives, we find the closest past odometry snapshot, apply the Kalman correction
 * there, then re-apply all subsequent odometry deltas forward to the present.
 */
public class SwervePosition {
    // Public-facing outputs
    private static Vector2 position    = new Vector2(0, 0);
    private static Vector2 absVelocity = new Vector2(0, 0);
    private static Vector2 lastOdomPos = new Vector2(0, 0);

    /**
     * Current best-estimate state [x, y].
     */
    private static Matrix<N2, N1> stateEstimate = VecBuilder.fill(0, 0);

    /**
     * Covariance matrix (P). Higher → less certain of position.
     */
    private static Matrix<N2, N2> uncertainty = Matrix.eye(Nat.N2()).times(0.1);

    /** How much uncertainty grows per meter of odometry travel. */
    private static final double ODOM_TRUST_COEFFICIENT = 10000;

    /** Vision measurement noise (R). Lower → trust vision more. */
    private static final Matrix<N2, N2> R_VISION = Matrix.eye(Nat.N2()).times(0.0000000000001);

    /**
     * History of odometry snapshots, keyed by timestamp (seconds).
     * Each entry stores [x, y] of the state estimate at that moment.
     * TreeMap keeps entries sorted by time so we can do floorEntry() lookups.
     */
    private static final TreeMap<Double, Matrix<N2, N1>> poseHistory = new TreeMap<>();

    /** How long (seconds) to retain history for retroactive correction. */
    private static final double HISTORY_WINDOW = 0.5;

    public static void init() {
        position      = new Vector2(0, 0);
        absVelocity   = new Vector2(0, 0);
        lastOdomPos   = new Vector2(0, 0);
        stateEstimate = VecBuilder.fill(0, 0);
        uncertainty   = Matrix.eye(Nat.N2()).times(0.1);
        poseHistory.clear();

        Odometry.init();
    }

    /**
     * Main update loop — call this every robot periodic tick.
     */
    public static void update() {
        double now = RTime.now();

        // 1. Compute odometry delta since last tick
        Vector2 currentOdomPos  = Odometry.getPosition();
        Vector2 odomDelta       = currentOdomPos.sub(lastOdomPos);
        double  distanceTraveled = odomDelta.mag();

        // 2. Predict: advance state with odometry
        predict(odomDelta, distanceTraveled);

        // 3. Prune stale history
        while (!poseHistory.isEmpty() && (now - poseHistory.firstKey()) > HISTORY_WINDOW) {
            poseHistory.pollFirstEntry();
        }

        // 4. Store current estimate in history (BEFORE vision correction)
        // Clone so later mutations to stateEstimate don't corrupt the stored snapshot.
        poseHistory.put(now, stateEstimate.copy());

        // //5. Retroactive vision correction 
        // Optional<Matrix<N2, N1>> visionMeasurement = VisionManager.getMatrixPosition();
        // if (visionMeasurement.isPresent()) {
        //     retroactiveCorrect(visionMeasurement.get(), VisionManager.getTimestampSeconds());
        // }

        //6. Publish outputs
        position = new Vector2(stateEstimate.get(0, 0), stateEstimate.get(1, 0));

        double dt = RTime.deltaTime();
        absVelocity = (dt > 0) ? odomDelta.div(dt) : new Vector2(0, 0);

        lastOdomPos = currentOdomPos;
    }

    /**
     * Prediction step: move estimate by the odometry delta and grow uncertainty.
     */
    private static void predict(Vector2 delta, double distance) {
        stateEstimate = stateEstimate.plus(VecBuilder.fill(delta.x, delta.y));

        // Uncertainty grows proportional to distance traveled (avoids blow-up at rest)
        Matrix<N2, N2> dynamicProcessNoise = Matrix.eye(Nat.N2()).times(distance * ODOM_TRUST_COEFFICIENT);
        uncertainty = uncertainty.plus(dynamicProcessNoise);
    }

    /**
     * Correction step: fuse a measurement into the current stateEstimate.
     * Returns the innovation (residual) so the caller can re-apply it if needed.
     */
    private static Matrix<N2, N1> correct(Matrix<N2, N1> estimate, Matrix<N2, N1> measurement) {
        // Kalman gain: K = P / (P + R)
        Matrix<N2, N2> kalmanGain = uncertainty.times(uncertainty.plus(R_VISION).inv());

        // Innovation: difference between vision and prediction
        Matrix<N2, N1> innovation = measurement.minus(estimate);

        // Corrected estimate: x = x + K * innovation
        Matrix<N2, N1> corrected = estimate.plus(kalmanGain.times(innovation));

        // Reduce uncertainty: P = (I - K) * P
        uncertainty = Matrix.eye(Nat.N2()).minus(kalmanGain).times(uncertainty);

        // Update the live stateEstimate with the same delta
        Matrix<N2, N1> delta = corrected.minus(estimate);
        stateEstimate = stateEstimate.plus(delta);

        return delta; 
    }

    /**
     * Retroactive correction:
     *  1. Find the history snapshot nearest to (and at-or-before) the vision timestamp.
     *  2. Apply the Kalman correction to that historical state.
     *  3. Propagate the correction delta forward through all subsequent history entries
     *     and into the live stateEstimate.
     *
     * This keeps the whole trajectory consistent with the vision fix.
     */
    private static void retroactiveCorrect(Matrix<N2, N1> measurement, double visionTimestamp) {
        if (poseHistory.isEmpty()) {
            // No history yet — just correct in place
            correct(stateEstimate.copy(), measurement);
            return;
        }

        // Find the snapshot closest to when the vision frame was actually captured
        Map.Entry<Double, Matrix<N2, N1>> entry = poseHistory.floorEntry(visionTimestamp);
        if (entry == null) {
            // Vision timestamp is older than everything we have — use oldest entry
            entry = poseHistory.firstEntry();
        }

        double historicalTimestamp = entry.getKey();
        Matrix<N2, N1> historicalEstimate = entry.getValue();

        // Apply the Kalman update at the historical snapshot; capture the delta
        Matrix<N2, N1> correctionDelta = correct(historicalEstimate, measurement);

        // Propagate the same delta to every subsequent history snapshot
        for (Map.Entry<Double, Matrix<N2, N1>> future : poseHistory.tailMap(historicalTimestamp, false).entrySet()) {
            future.setValue(future.getValue().plus(correctionDelta));
        }

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
        position      = newPosition;
        lastOdomPos   = newPosition;
        poseHistory.clear();

        // High confidence since we were explicitly told our position
        uncertainty = Matrix.eye(Nat.N2()).times(0.01);
    }

    public static Pose2d getPose() {
        return new Pose2d(
            new Translation2d(position.x, position.y),
            Rotation2d.fromRadians(Pigeon.getRotationRad())
        );
    }
}