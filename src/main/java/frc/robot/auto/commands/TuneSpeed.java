package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.Odometry;
import frc.robot.swerve.SwerveManager;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;

public class TuneSpeed extends Command {
    private double motorPercentOutput;
    private double time;
    private double startTime;
    private Vector2[] posFrames;
    
    public TuneSpeed(double motorPercentOutput, double time){
        this.motorPercentOutput = motorPercentOutput;
        this.time = time;
    }

    @Override
    public void initialize() {
       startTime = Timer.getFPGATimestamp();
       posFrames = new Vector2[4];

       posFrames[0] = new Vector2();
       posFrames[1] = new Vector2();
       posFrames[2] = new Vector2();
       posFrames[3] = new Vector2();
    }

    @Override
    public void execute() {
       SwerveManager.rotateAndDrive(0, new Vector2(motorPercentOutput, 0));

       Vector2 currentPos = Odometry.getPosition();

       posFrames[0] = posFrames[1];
       posFrames[1] = posFrames[2];
       posFrames[2] = posFrames[3];
       posFrames[3] = currentPos;

       Vector2 diffOne = posFrames[1].sub(posFrames[2]);
       Vector2 diffTwo = posFrames[2].sub(posFrames[3]);

       Vector2 velocity = diffOne.add(diffTwo).div(2).div(RTime.deltaTime());

       SmartDashboard.putNumber("VEL_X", velocity.x);
       SmartDashboard.putNumber("VEL_Y", velocity.x);

    }


    @Override
    public void end(boolean interrupted) {
        SwerveManager.rotateAndDrive(0, new Vector2());
    }

    @Override
    public boolean isFinished() {
        return startTime + time < Timer.getFPGATimestamp();
    }
}
    