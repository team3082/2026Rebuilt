// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.Pigeon;
import frc.robot.swerve.SwerveManager;
import frc.robot.vision.VisionManager;
import frc.robot.utils.Vector2;

public class RotateToTag extends Command {
  private final double speed;
  private final boolean moveRight;
  private boolean finished = false;

  /** Move with given lateralOutput until VisionManager reports a tag.
   *
   * @param speed magnitude of lateral speed to apply while searching (use small values, e.g. 0.15-0.25)
   * @param moveRight if true move right, if false move left
   */
  public RotateToTag(double speed, boolean moveRight) {
    this.speed = Math.abs(speed);
    this.moveRight = moveRight;
  }


  @Override
  public void initialize() {
    finished = false;
  }

  @Override
  public void execute() {
    // check for tag using cached VisionManager results and current pigeon yaw
    Optional<Vector2> pos = VisionManager.getPosition(Pigeon.getRotationRad());
    if (pos.isPresent()) {
      // stop and finish when a tag is visible
      SwerveManager.rotateAndDrive(0, new Vector2(0, 0));
      finished = true;
      return;
    }

    // still no tag: nudge to the chosen side
    double applied = moveRight ? speed : -speed;
    SwerveManager.rotateAndDrive(applied, new Vector2());
  }

  @Override
  public void end(boolean interrupted) {
    // ensure we stop moving laterally
    SwerveManager.rotateAndDrive(0, new Vector2(0, 0));
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
