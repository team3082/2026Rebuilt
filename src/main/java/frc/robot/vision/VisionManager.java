package frc.robot.vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.utils.Vector2;

public class VisionManager {
    private static Camera[] cameras = new Camera[0];
    private static boolean enabled = true;

    // Cache the latest results so each camera is only polled once per cycle (Bug 5)
    private static PhotonPipelineResult[] latestResults = new PhotonPipelineResult[0];

    public static void init() {
        if (Robot.isReal()) {
            cameras = new Camera[] {
                new Camera(new PhotonCamera("ApriltagCamera3"), new Vector2(11.0, 8.5), 0, Math.toRadians(15.0), -Math.PI/2),
                new Camera(new PhotonCamera("ApriltagCamera4"), new Vector2(-4.25, 13.25), Math.toRadians(15.0), 0, 0)
            };
            latestResults = new PhotonPipelineResult[cameras.length];
        }
    }

    /**
     * Call once per periodic tick to snapshot all camera results.
     * All other methods read from this cache instead of hitting the network repeatedly.
     */
    public static void poll() {
        for (int i = 0; i < cameras.length; i++) {
            latestResults[i] = cameras[i].photonCamera.getLatestResult();
        }
    }

    public static Optional<Vector2> getPosition(double pigeonAngle) {
        // Bug 3: respect the enabled flag
        if (!enabled) return Optional.empty();

        List<Vector2> positions = new ArrayList<>();

        for (int i = 0; i < cameras.length; i++) {
            Camera camera = cameras[i];
            if (camera.isDisabled()) continue;

            // Bug: use cached result, not a fresh network fetch
            PhotonPipelineResult result = latestResults[i];
            if (result == null) continue;

            PhotonTrackedTarget target = result.getBestTarget();

            // Bug: null check first, then duplicate check, then register
            if (target == null) continue;
            if (camera.isLatestTarget(target)) continue;
            camera.setLatestTarget(target);

            Transform3d transform = target.getBestCameraToTarget();
            int id = target.getFiducialId();

            // Bug: was `id > length`, should be `id >= length` (off-by-one)
            if (id < 0 || id >= Constants.APRIL_TAGS.length) continue;

            if (target.getPoseAmbiguity() > 0.2) continue;

            Vector2 vectorTransform = new Vector2(transform.getX(), transform.getY());
            vectorTransform = vectorTransform.rotate(camera.cameraYaw);

            double xdistRobot = vectorTransform.x * Math.cos(camera.cameraPitch) - transform.getZ() * Math.sin(camera.cameraPitch);
            double ydistRobot = vectorTransform.y * Math.cos(camera.cameraRoll)   + transform.getZ() * Math.sin(camera.cameraRoll);

            Vector2 distRobot = new Vector2(xdistRobot, ydistRobot);

            double xdistField = (Math.cos(pigeonAngle) * distRobot.x - Math.sin(pigeonAngle) * distRobot.y) * Constants.METERSTOINCHES;
            double ydistField = (Math.cos(pigeonAngle) * distRobot.y + Math.sin(pigeonAngle) * distRobot.x) * Constants.METERSTOINCHES;

            Vector2 cameraToTag = new Vector2(xdistField, ydistField);

            Vector2 aprilTagPos = new Vector2(Constants.APRIL_TAGS[id].getPosition().y, -Constants.APRIL_TAGS[id].getPosition().x);
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                aprilTagPos = aprilTagPos.rotate(Math.PI);
            }

            Vector2 cameraPos = aprilTagPos.sub(cameraToTag);
            Vector2 robotPos  = cameraPos.sub(camera.robotToCamera.rotate(pigeonAngle - (Math.PI / 2.0)));

            positions.add(robotPos);
        }

        if (positions.isEmpty()) return Optional.empty();

        double sumX = 0, sumY = 0;
        for (Vector2 p : positions) {
            sumX += p.x;
            sumY += p.y;
        }

        return Optional.of(new Vector2(sumX / positions.size(), sumY / positions.size()));
    }

    // Bug 2: cache getPosition() result instead of calling it twice
    public static Optional<Matrix<N2, N1>> getMatrixPosition() {
        double pigeonAngle = Pigeon.getRotationRad();
        Optional<Vector2> pos = getPosition(pigeonAngle);

        if (pos.isPresent()) {
            Vector2 position = pos.get();
            Matrix<N2, N1> mat = new Matrix<>(Nat.N2(), Nat.N1());
            mat.set(0, 0, position.x);
            mat.set(1, 0, position.y);
            return Optional.of(mat);
        }

        return Optional.empty();
    }

    public static Optional<Double> getRotation(double pigeonAngle) {
        if (!enabled) return Optional.empty();

        List<Double> robotYaws = new ArrayList<>();

        for (int i = 0; i < cameras.length; i++) {
            Camera camera = cameras[i];
            if (camera.isDisabled()) continue;

            // Bug 5: use cached result
            PhotonPipelineResult result = latestResults[i];
            if (result == null) continue;

            PhotonTrackedTarget target = result.getBestTarget();
            if (target == null) continue;

            Transform3d transform = target.getBestCameraToTarget();
            Rotation3d rotationTransform = transform.getRotation();

            robotYaws.add(rotationTransform.getZ() + camera.cameraYaw);
        }

        if (robotYaws.isEmpty()) return Optional.empty();

        double average = robotYaws.stream().mapToDouble(Double::doubleValue).average().getAsDouble();
        return Optional.of(average);
    }

    // Bug 4: only average timestamp/latency over cameras that have a valid result
    public static double getTimestampSeconds() {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < cameras.length; i++) {
            if (cameras[i].isDisabled() || latestResults[i] == null) continue;
            sum += latestResults[i].getTimestampSeconds();
            count++;
        }
        return count > 0 ? sum / count : 0;
    }

    public static double getLatency() {
        double sum = 0;
        int count = 0;
        for (int i = 0; i < cameras.length; i++) {
            if (cameras[i].isDisabled() || latestResults[i] == null) continue;
            sum += latestResults[i].metadata.getLatencyMillis() / 1000.0;
            count++;
        }
        return count > 0 ? sum / count : 0;
    }

    public static void enableVision()  { enabled = true; }
    public static void disableVision() { enabled = false; }
    public static boolean isEnabled()  { return enabled; }

    public static void disableLeftCam() {
        System.out.println("Disabled left camera");
        if (Robot.isReal()) cameras[0].disable();
    }

    public static void disableRightCam() {
        System.out.println("Disabled right camera");
        if (Robot.isReal()) cameras[1].disable();
    }

    public static void enableLeftCam() {
        System.out.println("Enabled left camera");
        if (Robot.isReal()) cameras[0].enable();
    }

    public static void enableRightCam() {
        System.out.println("Enabled right camera");
        if (Robot.isReal()) cameras[1].enable();
    }
}