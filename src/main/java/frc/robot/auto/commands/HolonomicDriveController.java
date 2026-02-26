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
     * Calculate the control output for the current time
     * @return Vector2 representing the velocity command (as percentage -1 to 1)
     */
    public Vector2 calculate() {
        double currentTime = RTime.now() - startTime;
        
        // Check if path is complete
        if (currentTime >= path.getDuration()) {
            isFinished = true;
            return new Vector2(0, 0);
        }
        
        // Get desired state from path
        ProfiledPoint desiredPoint = path.getPointAtTime(currentTime);
        Vector2 desiredPos = desiredPoint.getPosition();
        Vector2 desiredVel = desiredPoint.getVelocity().rotate(-Math.PI/2);

        double xFF = desiredVel.x / (Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL * 1.1);
        double yFF = desiredVel.y / (Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL * 1.1);
        
        // Get current robot pose
        Vector2 currentPos = SwervePosition.getPosition();

        // Calculate feedback velocities (based on position error).
        double xFeedback = xPositionPID.calculate(currentPos.x, desiredPos.x);
        double yFeedback = yPositionPID.calculate(currentPos.y, desiredPos.y);

        Vector2 feedbackVector = new Vector2(xFeedback, yFeedback).rotate(-Math.PI/2);
        

        SmartDashboard.putNumber("Holonomic/xPosError", desiredPos.x - currentPos.x);
        SmartDashboard.putNumber("Holonomic/yPosError", desiredPos.y - currentPos.y);
        SmartDashboard.putNumber("Holonomic/xFF", xFF);
        SmartDashboard.putNumber("Holonomic/yFF", yFF);
        
        return feedbackVector.add(new Vector2(xFF, yFF));
    }

    /**
     * Calculate the rotational control output for the current time.
     * Combines a heading feedforward (profiled rotational velocity) with a
     * heading-error feedback PID.
     *
     * @return Rotational speed command as a percentage (-1 to 1), suitable for
     *         passing directly as the {@code rotSpeed} argument of
     *         {@link frc.robot.swerve.SwerveManager#rotateAndDrive}.
     */
    public double calculateRotation() {
        double currentTime = RTime.now() - startTime;
        if (currentTime >= path.getDuration()) {
            return 0.0;
        }

        ProfiledPoint desiredPoint = path.getPointAtTime(currentTime);
        double desiredHeading = desiredPoint.getHeading();
        double desiredRotVel  = desiredPoint.getRotationalVelocity();

        double currentHeading = Pigeon.getRotationRad();

        double rotFF = desiredRotVel / 2;
        rotFF = 0;

        double rotFeedback = rotationPID.calculate(currentHeading, desiredHeading);

        double rotOutput = rotFF + rotFeedback;

        SmartDashboard.putNumber("Holonomic/headingError", desiredHeading - currentHeading);
        SmartDashboard.putNumber("Holonomic/rotFF", rotFF);
        SmartDashboard.putNumber("Holonomic/rotFeedback", rotFeedback);

        return Math.max(-1.0, Math.min(1.0, rotOutput));
    }
    
    /**
     * Check if the path following is complete
     */
    public boolean isFinished() {
        return isFinished;
    }
    
    /**
     * Get the current position error
     */
    public Vector2 getPositionError() {
        if (path == null) return new Vector2(0, 0);
        
        double currentTime = RTime.now() - startTime;
        ProfiledPoint desiredPoint = path.getPointAtTime(currentTime);
        Vector2 desiredPos = desiredPoint.getPosition();
        Vector2 currentPos = SwervePosition.getPosition();
        
        return desiredPos.sub(currentPos);
    }
    
}