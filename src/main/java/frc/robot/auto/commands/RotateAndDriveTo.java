package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePID;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.Vector2;
import frc.robot.utils.auto.ChickenPlannable;


public class RotateAndDriveTo extends Command{

    private Vector2 pos;
    private double rot;

    private boolean rotationOnly;

    /**
     * Move and rotate to specified point
     * @param angle
     * @param position
     */
    public RotateAndDriveTo(double angle, Vector2 position){
        rot = angle;
        pos = position;
    }

    public RotateAndDriveTo(double angle){
        rotationOnly = true;
        rot = angle;
        pos = new Vector2();
    }
    

    @ChickenPlannable
    public RotateAndDriveTo(double angle, double x, double y){
        rot = angle;
        pos = new Vector2(x, y);
    }

    @Override
    public void initialize() {
        if(rotationOnly){
            SwervePID.setDestState(SwervePosition.getPosition(), rot);
            return;
        }

        SwervePID.setDestState(pos, rot);
    }

    @Override
    public void execute() {
        SwerveManager.rotateAndDrive(SwervePID.updateOutputRot(), SwervePID.updateOutputVel());
    }

    @Override
    public boolean isFinished() {
        return SwervePID.atDest() && SwervePID.atRot();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
    }
}