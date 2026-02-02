package frc.robot.auto.commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.TrapezoidalProfile;
import frc.robot.utils.Vector2;
import frc.robot.utils.TrapezoidalProfile.TrapezoidalPreset;
import frc.robot.utils.trajectories.FeatherEvent;
import frc.robot.utils.trajectories.PurePursuit;
import frc.robot.utils.trajectories.RobotPath;

/**
 * Command that follows a {@link RobotPath} using a {@link PurePursuit} algorithm
 * while scheduling {@link FeatherEvent}s at specified points along the path.
 * 
 * <p>This class continuously updates the robot's drive vector based on its
 * current position along the path and triggers commands associated with events
 * when their corresponding path parameter {@code t} is reached. Supports
 * commands that run continuously on the path.
 */
public class FollowPath extends Command {
  private final PurePursuit follower;
  private PIDController profile = new PIDController(0.7, 0, 0.1, 0.05, 0, 1);
  private final RobotPath path;
  private final FeatherEvent[] events;  

  private int eventIndex = 0;
  
  /**
   * Constructs a FollowPath command.
   *
   * @param path   The {@link RobotPath} to follow.
   * @param events An array of {@link FeatherEvent}s to trigger along the path.
   *               Events must be ordered by their {@code t} values (progress along the path).
   */
  public FollowPath(RobotPath path, FeatherEvent[] events) {
    this.path = path;
    this.follower = new PurePursuit(path);
    this.events = events;
  }

  /**
   * Initializes the path follower and resets the event index.
   * Called once when the command is first scheduled.
   */
  @Override
  public void initialize() {
    follower.reset();
    System.out.println("FollowPath: Starting path following.");
    this.profile.setDest(1);
    eventIndex = 0;
  }

  /**
   * Called periodically while the command is scheduled.
   * 
   * <p>This method:
   * <ul>
   *   <li>Updates the path follower with the current robot position.</li>
   *   <li>Computes a drive vector based on a lookahead distance and sends it to the swerve drive.</li>
   *   <li>Schedules any {@link FeatherEvent} whose {@code t} value has been reached along the path.</li>
   * </ul>
   */
  @Override
  public void execute() {
    Vector2 robotPosition = SwervePosition.getPosition();
    follower.update(robotPosition);

    double t = follower.getCurrentT();
    double distance = path.getLengthAt(t);

    double velocity = profile.updateOutput(distance/path.getTotalLength());
    double lookAheadDistance = Tuning.AUTO_PATH_LOOKAHEAD_MIN + 
        (Tuning.AUTO_PATH_LOOKAHEAD_MAX - Tuning.AUTO_PATH_LOOKAHEAD_MIN) * velocity;

    Vector2 direction = follower.getDriveVector(robotPosition, lookAheadDistance);
    Vector2 driveVector = direction.mul(velocity);

    SwerveManager.rotateAndDrive(0, driveVector);
    
    if (events == null) {
      return;
    }

    if (eventIndex < events.length) {
      FeatherEvent currentEvent = events[eventIndex];
      if (follower.getCurrentT() >=  currentEvent.t) {
        if (!currentEvent.command.isScheduled()) {
          currentEvent.command.schedule();
        }
        eventIndex++;
      }
    }
  }

  /**
   * Ends the command, stopping the robot if interrupted.
   * 
   * <p>If {@code interrupted} is true:
   * <ul>
   *   <li>The robot drive is stopped.</li>
   *   <li>Any scheduled {@link FeatherEvent} commands are canceled.</li>
   * </ul>
   *
   * @param interrupted True if the command was interrupted/canceled before finishing normally.
   */
  @Override
  public void end(boolean interrupted) {
    SwerveManager.rotateAndDrive(0, new Vector2());

    for (FeatherEvent event : events) {
      if (event.command.isScheduled()) {
        event.command.cancel();
      }
    }
  }

  /**
   * Determines whether the command has finished.
   * 
   * <p>The command is considered finished when:
   * <ul>
   *   <li>All {@link FeatherEvent} commands are finished.</li>
   *   <li>The path follower has reached the end of the path (current {@code t} >= .95).</li>
   * </ul>
   *
   * @return True if the path has been fully followed and all events are complete.
   */
  @Override
  public boolean isFinished() {
    return follower.getCurrentT() >= 0.97;
  }
}
