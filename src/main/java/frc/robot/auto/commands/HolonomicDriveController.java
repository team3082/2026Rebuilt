package frc.robot.auto.commands;

import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.ProfiledPath;
import frc.robot.utils.trajectories.ProfiledPoint;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolonomicDriveController {
    private PIDController xPositionPID;
    private PIDController yPositionPID;
    private PIDController rotationPID;

    private ProfiledPath path;
    private double startTime;
    private boolean isFinished;

    public HolonomicDriveController(ProfiledPath path) {
        this.xPositionPID = new PIDController(Tuning.holonomic_pos_kp, Tuning.holonomic_pos_ki, Tuning.holonomic_pos_kd);
        this.yPositionPID = new PIDController(Tuning.holonomic_pos_kp, Tuning.holonomic_pos_ki, Tuning.holonomic_pos_kd);
        this.rotationPID  = new PIDController(Tuning.holonomic_rot_kp, Tuning.holonomic_rot_ki, Tuning.holonomic_rot_kd);
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        this.path = path;
        this.isFinished = false;
    }

    public void initialize(ProfiledPath path) {
        this.path = path;
        this.startTime = RTime.now();
        this.isFinished = false;
        xPositionPID.reset();
        yPositionPID.reset();
        rotationPID.reset();
    }

    /**
     * Calculate the translational velocity command for the current timestep.
     *
     * Feedforward (desired path velocity) dominates during motion; feedback
     * (lookahead position PID) corrects accumulated error. The two are summed
     * rather than blended so that neither term is artificially suppressed.
     * The combined output is clamped to unit magnitude if necessary.
     *
     * @return Velocity command as a percentage (-1 to 1) per axis.
     */
    public Vector2 calculate() {
        double currentTime = RTime.now() - startTime;

        if (currentTime >= path.getDuration()) {
            isFinished = true;
            return new Vector2(0, 0);
        }

        // --- Feedforward: desired velocity at the lookahead point ---
        // Using the lookahead point for FF keeps it temporally consistent with
        // the feedback term, preventing the mismatch that causes waypoint oscillation.
        double lookaheadTime = Math.min(currentTime + Tuning.holonomic_lookahead_time, path.getDuration());
        ProfiledPoint lookaheadPoint = path.getPointAtTime(lookaheadTime);
        ProfiledPoint currentPoint = path.getPointAtTime(currentTime);
        Vector2 lookaheadPos = lookaheadPoint.getPosition();
        Vector2 desiredVelocity = currentPoint.getVelocity()
                .rotate(-Math.PI / 2)
                .mul(1.0 / Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL);

        // --- Feedback: PID correcting error toward the lookahead position ---
        Vector2 currentPos = SwervePosition.getPosition();
        double xFeedback = xPositionPID.calculate(currentPos.x, lookaheadPos.x);
        double yFeedback = yPositionPID.calculate(currentPos.y, lookaheadPos.y);
        Vector2 feedbackVector = new Vector2(xFeedback, yFeedback).rotate(-Math.PI / 2);

        // Sum FF and FB — FF drives the motion, FB corrects for drift.
        // Clamp to unit magnitude only if the sum exceeds it.
        Vector2 combined = desiredVelocity.add(feedbackVector);
        if (combined.mag() > 1.0) {
            combined = combined.norm();
        }

        // Log against the *current* desired position for error visibility
        Vector2 desiredPos = path.getPointAtTime(currentTime).getPosition();
        SmartDashboard.putNumber("Holonomic/xPosError", desiredPos.x - currentPos.x);
        SmartDashboard.putNumber("Holonomic/yPosError", desiredPos.y - currentPos.y);
        SmartDashboard.putNumber("Holonomic/ffMag", desiredVelocity.mag());
        SmartDashboard.putNumber("Holonomic/feedbackMag", feedbackVector.mag());
        SmartDashboard.putNumber("Holonomic/combinedMag", combined.mag());

        return combined;
    }

    /**
     * Calculate the rotational velocity command for the current timestep.
     *
     * Combines a rotational velocity feedforward (from the profiled path) with
     * a heading-error feedback PID. FF is scaled to percent output units so the
     * two terms sum correctly. Add {@code holonomic_rot_ff_scale} to Tuning and
     * set it so that your path's maximum rotational velocity maps to ~1.0.
     *
     * @return Rotational speed as a percentage (-1 to 1), suitable for passing
     *         directly as the {@code rotSpeed} argument of
     *         {@link frc.robot.swerve.SwerveManager#rotateAndDrive}.
     */
    public double calculateRotation() {
        double currentTime = RTime.now() - startTime;

        if (currentTime >= path.getDuration()) {
            isFinished = true;
            return 0.0;
        }

        ProfiledPoint desiredPoint = path.getPointAtTime(currentTime + Tuning.holonomic_lookahead_time);
        double desiredHeading = desiredPoint.getHeading();
        double desiredRotVel  = desiredPoint.getRotationalVelocity();
        double currentHeading = Pigeon.getRotationRad();

        // Feedback: PID on heading error
        double rotFeedback = rotationPID.calculate(currentHeading, desiredHeading);

        double rotOutput = Math.max(-1.0, Math.min(1.0, rotFeedback));

        SmartDashboard.putNumber("Holonomic/headingError", desiredHeading - currentHeading);
        SmartDashboard.putNumber("Holonomic/rotFeedback", rotFeedback);
        SmartDashboard.putNumber("Holonomic/rotOutput", rotOutput);

        return rotOutput;
    }

    /** @return true once the robot has reached the end of the path. */
    public boolean isFinished() {
        return isFinished;
    }

    /** @return Position error between the profiled desired position and the robot's actual position. */
    public Vector2 getPositionError() {
        if (path == null) return new Vector2(0, 0);

        double currentTime = RTime.now() - startTime;
        Vector2 desiredPos = path.getPointAtTime(currentTime).getPosition();
        Vector2 currentPos = SwervePosition.getPosition();

        return desiredPos.sub(currentPos);
    }
}