package frc.robot.auto.commands;

import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.ProfiledPath;
import frc.robot.utils.trajectories.ProfiledPoint;
import frc.robot.Constants;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.RTime;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HolonomicDriveController {
    private PIDController xPositionPID;
    private PIDController yPositionPID;
    
    private ProfiledPath path;
    private double startTime;
    private boolean isFinished;


    public HolonomicDriveController(ProfiledPath path) {
        double posKp = 0.01; // Position PID constants
        double posKi = 0.005;
        double posKd = 0;

        this.xPositionPID = new PIDController(posKp, posKi, posKd);
        this.yPositionPID = new PIDController(posKp, posKi, posKd);
        
        this.path = path;
        this.isFinished = false;
    }

    public void initialize(ProfiledPath path) {
        this.path = path;
        this.startTime = RTime.now();
        this.isFinished = false;
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

        double xFF = desiredVel.x/(Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL);
        double yFF = desiredVel.y/(Constants.Swerve.PERCENT_OUT_TO_MOVE_VEL);
        
        // Get current robot pose
        Vector2 currentPos = SwervePosition.getPosition();

        // Calculate feedback velocities (based on position error).
        double xFeedback = xPositionPID.calculate(currentPos.x, desiredPos.x);
        double yFeedback = yPositionPID.calculate(currentPos.y, desiredPos.y);
        Vector2 feedbackVector = new Vector2(xFeedback, yFeedback).rotate(-Math.PI/2);


        SmartDashboard.putNumber("Error Magnitude", desiredPos.sub(currentPos).mag());
        SmartDashboard.putNumber("xFF", xFF);
        SmartDashboard.putNumber("yFF", yFF);
        SmartDashboard.putNumber("xFeedback", xFeedback);
        SmartDashboard.putNumber("yFeedback", yFeedback);
        
        return new Vector2(xFF, yFF  );
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