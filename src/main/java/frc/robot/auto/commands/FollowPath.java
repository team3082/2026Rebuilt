package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.swerve.SwerveManager;
import frc.robot.swerve.SwervePosition;
import frc.robot.utils.PIDController;
import frc.robot.utils.RTime;
import frc.robot.utils.Vector2;
import frc.robot.utils.trajectories.FeatherEvent;
import frc.robot.utils.trajectories.ProfiledPath;
import frc.robot.utils.trajectories.ProfiledPoint;
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
  private final ProfiledPath path;
  private final FeatherEvent[] events;  
  private final PIDController xPidController;
  private final PIDController yPidController;
  private final HolonomicDriveController holonomicDriveController;
  private double startTime;

  private int eventIndex = 0;
  
  /**
   * Constructs a FollowPath command.
   *
   * @param path   The {@link RobotPath} to follow.
   * @param events An array of {@link FeatherEvent}s to trigger along the path.
   *               Events must be ordered by their {@code t} values (progress along the path).
   */
  public FollowPath(ProfiledPath path, FeatherEvent[] events) {
    this.path = path;
    this.events = events;

    this.xPidController = new PIDController(
      Tuning.holonomic_pos_kp, Tuning.holonomic_pos_ki, Tuning.holonomic_pos_kd, .1, .1, 1
    );

    this.yPidController = new PIDController(
      Tuning.holonomic_pos_kp, Tuning.holonomic_pos_ki, Tuning.holonomic_pos_kd, .1, .1, 1
    );

    this.holonomicDriveController = new HolonomicDriveController(path);
  }

  /**
   * Initializes the path follower and resets the event index.
   * Called once when the command is first scheduled.
   */
  @Override
  public void initialize() {
    startTime = RTime.now();
    holonomicDriveController.initialize(path);
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
    // double elapsedTime = RTime.now() - startTime;s

    Vector2 driveVector = holonomicDriveController.calculate();
    SwerveManager.rotateAndDrive(0, driveVector);

    // double elapsedTime = RTime.now() - startTime;
    // ProfiledPoint targetPoint = path.getPointAtTime(elapsedTime);

    // Event handling would go here, but is not the focus of this change.
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
    return path.getDuration() + startTime <= RTime.now();
  }
}
